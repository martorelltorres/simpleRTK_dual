// Copyright 2026 Antoni Martorell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ROS driver node for the u-blox ZED-X20D: configures the receiver over USB with a
// verified CFG-VALSET/CFG-VALGET sequence and publishes position, velocity and the raw
// NAV-DAHEADING / NAV-PVT / NAV-HPPOSLLH messages. Optionally logs every UBX frame to
// disk for PPK post-processing.
//
// Threads: the serial read thread runs the framer, the frame handlers and the ROS
// publishers. The ROS timer thread runs the configuration engine tick, executes its
// actions, reopens the port and updates diagnostics. The engine is shared and guarded
// by engine_mutex_; actions are executed outside the lock.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <boost/filesystem/operations.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>
#include <rtcm_msgs/Message.h>

#include "ublox_x20d_driver/cfg_keys.h"
#include "ublox_x20d_driver/cfg_valset_valget.h"
#include "ublox_x20d_driver/config_engine.h"
#include "ublox_x20d_driver/nav_conversion.h"
#include "ublox_x20d_driver/nav_publisher.h"
#include "ublox_x20d_driver/raw_logger.h"
#include "ublox_x20d_driver/serial_transport.h"
#include "ublox_x20d_driver/ubx_framer.h"
#include "ublox_x20d_driver/ubx_types.h"

namespace ublox_x20d_driver
{
namespace
{
using SteadyClock = std::chrono::steady_clock;

// Measurement periods below this lose the dual-antenna heading on HDG 2.00.
constexpr int kMinHeadingRateMeasMs = 500;

std::string expand_home(const std::string& path)
{
  if (!path.empty() && path[0] == '~')
  {
    const char* home = std::getenv("HOME");
    return std::string(home ? home : "") + path.substr(1);
  }
  return path;
}

std::vector<uint8_t> value_bytes(const cfg::KeyValue& kv)
{
  std::vector<uint8_t> bytes;
  for (size_t i = 0; i < cfg::value_size(kv.key_id); ++i)
  {
    bytes.push_back(static_cast<uint8_t>(kv.raw >> (8 * i)));
  }
  return bytes;
}
}  // namespace

class UbloxX20dNode
{
public:
  UbloxX20dNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : pnh_(pnh)
    , transport_([this](const uint8_t* d, size_t n) { on_data(d, n); },
                 [this](const std::string& what) { on_port_error(what); })
    , framer_([this](const ubx::Frame& f) { on_frame(f); })
  {
    (void)nh;
    if (!load_parameters())
    {
      ros::shutdown();
      return;
    }

    nav_publisher_.reset(new NavPublisher(pnh_, frame_id_, publish_gga_));
    if (rtcm_input_)
    {
      rtcm_sub_ = pnh_.subscribe("rtcm", 32, &UbloxX20dNode::on_rtcm, this);
    }

    updater_.setHardwareID("ublox_x20d " + device_);
    updater_.add("link", this, &UbloxX20dNode::diagnose_link);
    updater_.add("config", this, &UbloxX20dNode::diagnose_config);
    updater_.add("fix", this, &UbloxX20dNode::diagnose_fix);
    updater_.add("heading", this, &UbloxX20dNode::diagnose_heading);
    updater_.add("raw_log", this, &UbloxX20dNode::diagnose_raw_log);

    try_open();
    timer_ = pnh_.createTimer(ros::Duration(0.05), &UbloxX20dNode::on_timer, this);
  }

  ~UbloxX20dNode()
  {
    transport_.close();
  }

private:
  // --- parameters and desired configuration ----------------------------------------

  bool load_parameters()
  {
    pnh_.param<std::string>("device", device_, "/dev/ttyACM0");
    pnh_.param("baud_rate", baud_rate_, 460800);
    pnh_.param<std::string>("frame_id", frame_id_, "gps");
    pnh_.param("reopen_period", reopen_period_s_, 1.0);
    pnh_.param("publish_gga", publish_gga_, false);

    int rate_meas_ms = 1000;
    int rate_nav = 1;
    int dyn_model = static_cast<int>(cfg::kDynModelSea);
    double receiver_heading_offset_deg = 0.0;
    bool nmea_output = false;
    bool allow_receiver_reset = true;
    pnh_.param("rate_meas_ms", rate_meas_ms, rate_meas_ms);
    pnh_.param("rate_nav", rate_nav, rate_nav);
    pnh_.param("dyn_model", dyn_model, dyn_model);
    pnh_.param("receiver_heading_offset_deg", receiver_heading_offset_deg, receiver_heading_offset_deg);
    pnh_.param("nmea_output", nmea_output, nmea_output);
    pnh_.param("rtcm_input", rtcm_input_, rtcm_input_);
    pnh_.param("allow_receiver_reset", allow_receiver_reset, allow_receiver_reset);

    pnh_.param("enable_raw_observables", raw_enabled_, false);
    std::string raw_dir;
    std::string raw_prefix;
    std::string raw_content;
    double raw_rotate_minutes = 60.0;
    pnh_.param<std::string>("raw_log_dir", raw_dir, "~/.ros/ubx");
    pnh_.param<std::string>("raw_log_prefix", raw_prefix, "x20d");
    pnh_.param<std::string>("raw_log_content", raw_content, "all");
    pnh_.param("raw_log_rotate_minutes", raw_rotate_minutes, raw_rotate_minutes);

    if (rate_meas_ms < 25 || rate_meas_ms > 65535 || rate_nav < 1 || rate_nav > 127)
    {
      ROS_FATAL("rate_meas_ms must be in [25, 65535] and rate_nav in [1, 127]");
      return false;
    }
    const long offset_cdeg = std::lround(receiver_heading_offset_deg * 100.0);
    if (std::abs(offset_cdeg) > cfg::kDAHeadingOffsetLimit)
    {
      ROS_FATAL("receiver_heading_offset_deg must be within +/-180 deg");
      return false;
    }
    if (raw_content != "all" && raw_content != "rxm_only")
    {
      ROS_FATAL("raw_log_content must be 'all' or 'rxm_only'");
      return false;
    }
    if (rate_meas_ms < kMinHeadingRateMeasMs)
    {
      ROS_WARN("rate_meas_ms = %d: below %d ms the dual-antenna heading is not resolved "
               "on HDG 2.00 firmware",
               rate_meas_ms, kMinHeadingRateMeasMs);
      heading_rate_warning_ = true;
    }

    auto set = [this](const cfg::CfgKey& key, int64_t value) {
      desired_[key.id] = cfg::encode_value(key.id, value);
      key_names_[key.id] = key.name;
    };
    namespace k = cfg::keys;
    set(k::kUsbInProtUbx, 1);
    set(k::kUsbOutProtUbx, 1);
    set(k::kUsbOutProtNmea, nmea_output ? 1 : 0);
    set(k::kUsbInProtRtcm3x, rtcm_input_ ? 1 : 0);
    set(k::kRateMeas, rate_meas_ms);
    set(k::kRateNav, rate_nav);
    set(k::kNavSpgDynModel, dyn_model);
    set(k::kNavSpgDAHeadingOffset, offset_cdeg);
    set(k::kMsgOutNavDAHeadingUsb, 1);
    set(k::kMsgOutNavPvtUsb, 1);
    set(k::kMsgOutNavHPPosLLHUsb, 1);
    set(k::kMsgOutRxmRawxUsb, raw_enabled_ ? 1 : 0);
    set(k::kMsgOutRxmSfrbxUsb, raw_enabled_ ? 1 : 0);

    start_info_.user_keys.clear();
    for (const auto& entry : desired_)
    {
      start_info_.user_keys.push_back({ entry.first, key_names_[entry.first] });
    }
    start_info_.nav_msgout_enabled = true;
    start_info_.nmea_disabled = !nmea_output;
    start_info_.nav_period_s = rate_meas_ms * rate_nav / 1000.0;

    cfgeng::Params params;
    if (!allow_receiver_reset)
    {
      params.reset_attempts = 0;
    }
    engine_.set_params(params);

    if (raw_enabled_)
    {
      boost::system::error_code ec;
      boost::filesystem::create_directories(expand_home(raw_dir), ec);
      if (ec)
      {
        ROS_FATAL("cannot create raw_log_dir %s: %s", expand_home(raw_dir).c_str(), ec.message().c_str());
        return false;
      }
      raw_logger_.reset(new RawLogger(expand_home(raw_dir), raw_prefix, raw_rotate_minutes,
                                      raw_content == "all" ? RawLogger::Content::kAll :
                                                             RawLogger::Content::kRxmOnly));
      ROS_INFO("raw UBX logging to %s", expand_home(raw_dir).c_str());
    }
    return true;
  }

  // --- serial port -------------------------------------------------------------------

  void try_open()
  {
    last_open_attempt_ = SteadyClock::now();
    std::string error;
    if (!transport_.open(device_, static_cast<unsigned int>(baud_rate_), &error))
    {
      if (!open_failure_reported_)
      {
        ROS_ERROR("cannot open %s (%s); retrying every %.1f s", device_.c_str(), error.c_str(), reopen_period_s_);
        open_failure_reported_ = true;
      }
      return;
    }
    open_failure_reported_ = false;
    ROS_INFO("opened %s", device_.c_str());
    std::lock_guard<std::mutex> lock(engine_mutex_);
    engine_.start(start_info_, SteadyClock::now());
  }

  // ROS thread: RTCM corrections (e.g. from an NTRIP client) go to the receiver as is.
  void on_rtcm(const rtcm_msgs::Message::ConstPtr& msg)
  {
    if (msg->message.empty())
    {
      return;
    }
    std::string error;
    if (!transport_.write(msg->message, &error))
    {
      ++rtcm_dropped_;
      ROS_WARN_THROTTLE(10.0, "RTCM not forwarded to the receiver: %s", error.c_str());
      return;
    }
    rtcm_bytes_ += msg->message.size();
    last_rtcm_steady_ = SteadyClock::now();
  }

  // Read thread.
  void on_port_error(const std::string& what)
  {
    ROS_WARN("serial port %s lost: %s", device_.c_str(), what.c_str());
    port_lost_ = true;
  }

  // Read thread.
  void on_data(const uint8_t* data, size_t length)
  {
    rx_stamp_ = ros::Time::now();
    last_rx_steady_ = SteadyClock::now();
    bytes_received_ += length;
    const uint64_t nmea_before = framer_.stats().nmea_sentences;
    framer_.feed(data, length);
    const uint64_t nmea_new = framer_.stats().nmea_sentences - nmea_before;
    if (nmea_new > 0)
    {
      std::lock_guard<std::mutex> lock(engine_mutex_);
      engine_.on_nmea(nmea_new, framer_.last_nmea(), SteadyClock::now());
    }
    std::lock_guard<std::mutex> lock(stats_mutex_);
    framer_stats_ = framer_.stats();
  }

  // --- frame handling (read thread) ------------------------------------------------------

  void on_frame(const ubx::Frame& frame)
  {
    if (raw_logger_)
    {
      std::lock_guard<std::mutex> lock(raw_mutex_);
      if (!raw_logger_->write(frame, std::chrono::system_clock::now()))
      {
        ROS_ERROR_THROTTLE(10.0, "raw log: %s", raw_logger_->error().c_str());
      }
    }

    const auto now = SteadyClock::now();
    switch (frame.msg_class)
    {
      case ubx::msg_class::kAck:
        handle_ack(frame, now);
        break;
      case ubx::msg_class::kCfg:
        if (frame.msg_id == ubx::msg_id::kCfgValget)
        {
          handle_valget(frame, now);
        }
        break;
      case ubx::msg_class::kMon:
        if (frame.msg_id == ubx::msg_id::kMonVer)
        {
          handle_mon_ver(frame, now);
        }
        break;
      case ubx::msg_class::kNav:
        if (nav_publisher_->handle(frame, rx_stamp_))
        {
          std::lock_guard<std::mutex> lock(engine_mutex_);
          engine_.on_nav_frame(now);
        }
        break;
      default:
        break;
    }
  }

  void handle_ack(const ubx::Frame& frame, SteadyClock::time_point now)
  {
    uint8_t cls = 0;
    uint8_t id = 0;
    if (!cfg::parse_ack(frame.payload, &cls, &id))
    {
      return;
    }
    std::lock_guard<std::mutex> lock(engine_mutex_);
    if (frame.msg_id == ubx::msg_id::kAckAck)
    {
      engine_.on_ack(cls, id, now);
    }
    else if (frame.msg_id == ubx::msg_id::kAckNak)
    {
      engine_.on_nak(cls, id, now);
    }
  }

  void handle_valget(const ubx::Frame& frame, SteadyClock::time_point now)
  {
    cfg::ValgetResponse response;
    if (!cfg::parse_valget_response(frame.payload, &response))
    {
      ROS_WARN("malformed CFG-VALGET response (%zu bytes)", frame.payload.size());
      return;
    }
    std::vector<cfgeng::KeyBytes> pairs;
    for (const cfg::KeyValue& kv : response.values)
    {
      pairs.push_back({ kv.key_id, value_bytes(kv) });
    }
    std::lock_guard<std::mutex> lock(engine_mutex_);
    engine_.on_valget_response(pairs, now);
  }

  void handle_mon_ver(const ubx::Frame& frame, SteadyClock::time_point now)
  {
    ubx::MonVer ver;
    if (ubx::parse_mon_ver(frame.payload, &ver) != ubx::ParseResult::kOk)
    {
      return;
    }
    const bool compatible = ubx::mon_ver_compatible(ver);
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      mon_ver_summary_ = ubx::mon_ver_summary(ver);
    }
    std::lock_guard<std::mutex> lock(engine_mutex_);
    engine_.on_mon_ver(compatible, ubx::mon_ver_summary(ver), now);
  }

  // --- timer (ROS thread) --------------------------------------------------------------

  void on_timer(const ros::TimerEvent&)
  {
    const auto now = SteadyClock::now();
    if (port_lost_.exchange(false))
    {
      transport_.close();
      framer_.reset();
      std::lock_guard<std::mutex> lock(engine_mutex_);
      engine_.on_detached(now);
    }
    if (!transport_.is_open() &&
        std::chrono::duration<double>(now - last_open_attempt_).count() >= reopen_period_s_)
    {
      try_open();
    }

    std::vector<cfgeng::Action> actions;
    {
      std::lock_guard<std::mutex> lock(engine_mutex_);
      actions = engine_.tick(now);
    }
    for (const cfgeng::Action& action : actions)
    {
      execute(action);
    }
    updater_.update();
  }

  void send(uint8_t msg_class, uint8_t msg_id, const std::vector<uint8_t>& payload, const char* what)
  {
    std::string error;
    if (!transport_.write(ubx::encode_frame(msg_class, msg_id, payload), &error))
    {
      std::lock_guard<std::mutex> lock(engine_mutex_);
      engine_.on_send_failed(std::string(what) + ": " + error, SteadyClock::now());
    }
  }

  void execute(const cfgeng::Action& action)
  {
    switch (action.type)
    {
      case cfgeng::ActionType::POLL_MON_VER:
        send(ubx::msg_class::kMon, ubx::msg_id::kMonVer, {}, "MON-VER poll");
        break;
      case cfgeng::ActionType::SEND_VALSET:
      {
        std::vector<cfg::KeyValue> values;
        std::vector<cfgeng::KeyBytes> expected;
        for (uint32_t key : action.keys)
        {
          const auto it = desired_.find(key);
          if (it == desired_.end())
          {
            continue;
          }
          const cfg::KeyValue kv{ key, it->second };
          values.push_back(kv);
          expected.push_back({ key, value_bytes(kv) });
        }
        send(ubx::msg_class::kCfg, ubx::msg_id::kCfgValset,
             cfg::build_valset_payload(values, static_cast<cfg::Transaction>(action.txn_action)), "CFG-VALSET");
        std::lock_guard<std::mutex> lock(engine_mutex_);
        engine_.on_valset_sent(expected, SteadyClock::now());
        break;
      }
      case cfgeng::ActionType::SEND_VALGET_VERIFY:
        send(ubx::msg_class::kCfg, ubx::msg_id::kCfgValget, cfg::build_valget_payload(action.keys, cfg::kValgetLayerRam),
             "CFG-VALGET");
        break;
      case cfgeng::ActionType::SEND_CFG_RST:
        send(ubx::msg_class::kCfg, ubx::msg_id::kCfgRst,
             cfg::build_cfg_rst_payload(cfg::kNavBbrHotStart, engine_params_reset_mode()), "CFG-RST");
        break;
      case cfgeng::ActionType::MARK_KEYS_VERIFIED:
        verified_keys_ = action.keys.size();
        break;
      case cfgeng::ActionType::MARK_KEYS_ACKNAK:
        for (uint32_t key : action.keys)
        {
          rejected_keys_.push_back(key_names_.count(key) ? key_names_[key] : std::to_string(key));
        }
        break;
      case cfgeng::ActionType::LOG:
        log(action.level, action.text);
        break;
    }
  }

  uint8_t engine_params_reset_mode()
  {
    std::lock_guard<std::mutex> lock(engine_mutex_);
    return engine_.params().reset_mode;
  }

  static void log(cfgeng::LogLevel level, const std::string& text)
  {
    switch (level)
    {
      case cfgeng::LogLevel::DEBUG:
        ROS_DEBUG("config: %s", text.c_str());
        break;
      case cfgeng::LogLevel::INFO:
        ROS_INFO("config: %s", text.c_str());
        break;
      case cfgeng::LogLevel::WARN:
        ROS_WARN("config: %s", text.c_str());
        break;
      case cfgeng::LogLevel::ERROR:
        ROS_ERROR("config: %s", text.c_str());
        break;
    }
  }

  // --- diagnostics (ROS thread) ----------------------------------------------------------

  void diagnose_link(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    ubx::UbxFramer::Stats s;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      s = framer_stats_;
    }
    const auto now = SteadyClock::now();
    const double dt = link_window_start_ == SteadyClock::time_point() ?
                          0.0 :
                          std::chrono::duration<double>(now - link_window_start_).count();
    const uint64_t bytes = bytes_received_;
    if (dt > 0.0)
    {
      stat.add("bytes/s", (bytes - link_prev_bytes_) / dt);
      stat.add("frames/s", (s.frames - link_prev_.frames) / dt);
      stat.add("NMEA sentences/s", (s.nmea_sentences - link_prev_.nmea_sentences) / dt);
    }
    stat.add("frames", s.frames);
    stat.add("checksum errors", s.checksum_errors);
    stat.add("length errors", s.length_errors);
    stat.add("device", device_);
    if (rtcm_input_)
    {
      stat.add("RTCM bytes forwarded", rtcm_bytes_);
      stat.add("RTCM messages dropped", rtcm_dropped_);
      if (rtcm_bytes_ > 0)
      {
        stat.add("last RTCM age (s)", std::chrono::duration<double>(now - last_rtcm_steady_).count());
      }
    }
    link_prev_ = s;
    link_prev_bytes_ = bytes;
    link_window_start_ = now;

    if (!transport_.is_open())
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "port not open");
      return;
    }
    const double age = std::chrono::duration<double>(now - last_rx_steady_.load()).count();
    stat.add("last data age (s)", age);
    if (age > 3.0)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "no data from receiver");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "receiving");
    }
  }

  void diagnose_config(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    cfgeng::State state;
    std::string rung;
    int resets;
    {
      std::lock_guard<std::mutex> lock(engine_mutex_);
      state = engine_.state();
      rung = cfgeng::to_string(engine_.rung());
      resets = engine_.reset_count();
    }
    std::string mon_ver;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      mon_ver = mon_ver_summary_;
    }
    stat.add("state", cfgeng::to_string(state));
    stat.add("rung", rung);
    stat.add("resets", resets);
    stat.add("verified keys", verified_keys_);
    stat.add("MON-VER", mon_ver);
    std::string rejected;
    for (const std::string& name : rejected_keys_)
    {
      rejected += (rejected.empty() ? "" : ", ") + name;
    }
    stat.add("rejected keys", rejected);

    switch (state)
    {
      case cfgeng::State::READY:
        if (!rejected_keys_.empty())
        {
          stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "verified, some keys rejected");
        }
        else if (heading_rate_warning_)
        {
          stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "verified; rate too high for heading");
        }
        else
        {
          stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "verified on device");
        }
        break;
      case cfgeng::State::DEGRADED:
      case cfgeng::State::INCOMPATIBLE:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, cfgeng::to_string(state));
        break;
      default:
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, cfgeng::to_string(state));
        break;
    }
  }

  void diagnose_fix(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    const NavPublisher::Snapshot nav = nav_publisher_->snapshot();
    const ubx::NavPvt& pvt = nav.pvt;
    if (!nav.have_pvt)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::STALE, "no NAV-PVT received");
      return;
    }
    stat.add("fix type", static_cast<int>(pvt.fix_type));
    stat.add("carrier solution", static_cast<int>(pvt.carr_soln));
    stat.add("satellites", static_cast<int>(pvt.num_sv));
    stat.add("hAcc (m)", pvt.h_acc * 1e-3);
    stat.add("vAcc (m)", pvt.v_acc * 1e-3);
    if (nav::nav_sat_status(pvt) == nav::kStatusNoFix)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "no fix");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "fix");
    }
  }

  void diagnose_heading(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    const NavPublisher::Snapshot nav = nav_publisher_->snapshot();
    const auto now = SteadyClock::now();
    const double dt = heading_window_start_ == SteadyClock::time_point() ?
                          0.0 :
                          std::chrono::duration<double>(now - heading_window_start_).count();
    if (dt > 0.0)
    {
      stat.add("rate (Hz)", (nav.daheading_count - heading_prev_count_) / dt);
    }
    heading_prev_count_ = nav.daheading_count;
    heading_window_start_ = now;

    if (!nav.have_daheading || std::chrono::duration<double>(now - nav.daheading_time).count() > 5.0)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::STALE, "no recent NAV-DAHEADING");
      return;
    }
    const ubx::NavDAHeading& m = nav.daheading;
    stat.add("heading valid", m.rel_pos_heading_valid);
    stat.add("carrier solution", static_cast<int>(m.carr_soln));
    stat.add("heading accuracy (deg)", m.acc_heading * 1e-5);
    stat.add("baseline length (m)", m.rel_pos_length * 1e-3);
    if (!m.gnss_fix_ok || !m.rel_pos_heading_valid)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "heading not valid");
    }
    else if (m.carr_soln != ubx::kCarrSolnFixed)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "heading valid, ambiguities not fixed");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "heading valid, fixed");
    }
  }

  void diagnose_raw_log(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (!raw_logger_)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "disabled");
      return;
    }
    std::lock_guard<std::mutex> lock(raw_mutex_);
    stat.add("file", raw_logger_->current_path());
    stat.add("bytes written", raw_logger_->bytes_written());
    if (!raw_logger_->error().empty())
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, raw_logger_->error());
    }
    else if (raw_logger_->current_path().empty())
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "no frame logged yet");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "logging");
    }
  }

  // --- state ---------------------------------------------------------------------------

  ros::NodeHandle pnh_;
  std::string device_;
  int baud_rate_ = 460800;
  std::string frame_id_;
  double reopen_period_s_ = 1.0;
  bool publish_gga_ = false;
  bool rtcm_input_ = true;
  bool raw_enabled_ = false;
  bool heading_rate_warning_ = false;

  std::map<uint32_t, uint64_t> desired_;
  std::map<uint32_t, std::string> key_names_;
  cfgeng::StartInfo start_info_;

  SerialTransport transport_;
  ubx::UbxFramer framer_;  // read thread only, except reset() while the port is closed
  std::mutex engine_mutex_;
  cfgeng::ConfigEngine engine_;
  std::mutex raw_mutex_;
  std::unique_ptr<RawLogger> raw_logger_;

  std::unique_ptr<NavPublisher> nav_publisher_;
  ros::Timer timer_;
  ros::Subscriber rtcm_sub_;
  diagnostic_updater::Updater updater_;

  std::atomic<bool> port_lost_{ false };
  SteadyClock::time_point last_open_attempt_;
  bool open_failure_reported_ = false;
  ros::Time rx_stamp_;
  std::atomic<SteadyClock::time_point> last_rx_steady_{ SteadyClock::time_point() };
  std::atomic<uint64_t> bytes_received_{ 0 };

  // Shared between the read thread and diagnostics; guarded by stats_mutex_.
  std::mutex stats_mutex_;
  ubx::UbxFramer::Stats framer_stats_;
  std::string mon_ver_summary_;

  // ROS-thread only.
  uint64_t rtcm_bytes_ = 0;
  uint64_t rtcm_dropped_ = 0;
  SteadyClock::time_point last_rtcm_steady_;
  size_t verified_keys_ = 0;
  std::vector<std::string> rejected_keys_;
  ubx::UbxFramer::Stats link_prev_;
  uint64_t link_prev_bytes_ = 0;
  SteadyClock::time_point link_window_start_;
  uint64_t heading_prev_count_ = 0;
  SteadyClock::time_point heading_window_start_;
};

}  // namespace ublox_x20d_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ublox_x20d");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ublox_x20d_driver::UbloxX20dNode node(nh, pnh);
  ros::spin();
  return 0;
}
