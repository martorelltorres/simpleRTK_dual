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

#include "ublox_x20d_driver/nav_publisher.h"

#include <algorithm>

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <ublox_x20d_msgs/NavDAHeading.h>
#include <ublox_x20d_msgs/NavHPPosLLH.h>
#include <ublox_x20d_msgs/NavPVT.h>
#include <ublox_x20d_msgs/RxmRawx.h>
#include <ublox_x20d_msgs/RxmSfrbx.h>

#include "ublox_x20d_driver/nav_conversion.h"
#include "ublox_x20d_driver/nmea_gga.h"

namespace ublox_x20d_driver
{

NavPublisher::NavPublisher(ros::NodeHandle& pnh, const std::string& frame_id, bool publish_gga,
                           bool publish_raw)
  : frame_id_(frame_id), publish_raw_(publish_raw)
{
  fix_pub_ = pnh.advertise<sensor_msgs::NavSatFix>("fix", 10);
  vel_pub_ = pnh.advertise<geometry_msgs::TwistWithCovarianceStamped>("vel", 10);
  pvt_pub_ = pnh.advertise<ublox_x20d_msgs::NavPVT>("nav_pvt", 10);
  hpposllh_pub_ = pnh.advertise<ublox_x20d_msgs::NavHPPosLLH>("nav_hpposllh", 10);
  daheading_pub_ = pnh.advertise<ublox_x20d_msgs::NavDAHeading>("nav_daheading", 10);
  if (publish_gga)
  {
    nmea_pub_ = pnh.advertise<nmea_msgs::Sentence>("nmea", 10);
  }
  if (publish_raw)
  {
    rawx_pub_ = pnh.advertise<ublox_x20d_msgs::RxmRawx>("rxm_rawx", 10);
    sfrbx_pub_ = pnh.advertise<ublox_x20d_msgs::RxmSfrbx>("rxm_sfrbx", 100);
  }
}

bool NavPublisher::handle(const ubx::Frame& frame, const ros::Time& stamp)
{
  if (frame.msg_class != ubx::msg_class::kNav)
  {
    return false;
  }
  stamp_ = stamp;
  switch (frame.msg_id)
  {
    case ubx::msg_id::kNavDAHeading:
      handle_daheading(frame);
      return true;
    case ubx::msg_id::kNavPvt:
      handle_pvt(frame);
      return true;
    case ubx::msg_id::kNavHPPosLLH:
      handle_hpposllh(frame);
      return true;
    default:
      return false;
  }
}

bool NavPublisher::handle_raw(const ubx::Frame& frame, const ros::Time& stamp)
{
  if (frame.msg_class != ubx::msg_class::kRxm)
  {
    return false;
  }
  switch (frame.msg_id)
  {
    case ubx::msg_id::kRxmRawx:
      if (publish_raw_)
      {
        stamp_ = stamp;
        handle_rawx(frame);
      }
      return true;
    case ubx::msg_id::kRxmSfrbx:
      if (publish_raw_)
      {
        stamp_ = stamp;
        handle_sfrbx(frame);
      }
      return true;
    default:
      return false;
  }
}

void NavPublisher::handle_rawx(const ubx::Frame& frame)
{
  ubx::RxmRawx m;
  if (ubx::parse_rxm_rawx(frame.payload, &m) != ubx::ParseResult::kOk)
  {
    ROS_WARN_THROTTLE(30.0, "RXM-RAWX payload too short (%zu bytes)", frame.payload.size());
    return;
  }
  ublox_x20d_msgs::RxmRawx msg;
  msg.header = header();
  msg.rcv_tow = m.rcv_tow;
  msg.week = m.week;
  msg.leap_s = m.leap_s;
  msg.rec_stat = m.rec_stat;
  msg.leap_sec = (m.rec_stat & 0x01u) != 0;
  msg.clk_reset = (m.rec_stat & 0x02u) != 0;
  msg.version = m.version;
  msg.reserved0 = m.reserved0;
  msg.meas.resize(m.meas.size());
  for (size_t i = 0; i < m.meas.size(); ++i)
  {
    const ubx::RawxMeas& r = m.meas[i];
    ublox_x20d_msgs::RawxMeas& out = msg.meas[i];
    out.pr_mes = r.pr_mes;
    out.cp_mes = r.cp_mes;
    out.do_mes = r.do_mes;
    out.gnss_id = r.gnss_id;
    out.sv_id = r.sv_id;
    out.sig_id = r.sig_id;
    out.freq_id = r.freq_id;
    out.locktime = r.locktime;
    out.cno = r.cno;
    out.pr_stdev = r.pr_stdev;
    out.cp_stdev = r.cp_stdev;
    out.do_stdev = r.do_stdev;
    out.trk_stat = r.trk_stat;
    out.pr_valid = (r.trk_stat & 0x01u) != 0;
    out.cp_valid = (r.trk_stat & 0x02u) != 0;
    out.half_cyc = (r.trk_stat & 0x04u) != 0;
    out.sub_half_cyc = (r.trk_stat & 0x08u) != 0;
  }
  rawx_pub_.publish(msg);
}

void NavPublisher::handle_sfrbx(const ubx::Frame& frame)
{
  ubx::RxmSfrbx m;
  if (ubx::parse_rxm_sfrbx(frame.payload, &m) != ubx::ParseResult::kOk)
  {
    ROS_WARN_THROTTLE(30.0, "RXM-SFRBX payload too short (%zu bytes)", frame.payload.size());
    return;
  }
  ublox_x20d_msgs::RxmSfrbx msg;
  msg.header = header();
  msg.gnss_id = m.gnss_id;
  msg.sv_id = m.sv_id;
  msg.sig_id = m.sig_id;
  msg.freq_id = m.freq_id;
  msg.chn = m.chn;
  msg.version = m.version;
  msg.reserved0 = m.reserved0;
  msg.dwrd = m.dwrd;
  sfrbx_pub_.publish(msg);
}

NavPublisher::Snapshot NavPublisher::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  Snapshot s;
  s.have_pvt = have_pvt_;
  s.pvt = last_pvt_;
  s.have_daheading = have_daheading_;
  s.daheading = last_daheading_;
  s.daheading_time = last_daheading_steady_;
  s.daheading_count = daheading_count_;
  return s;
}

std_msgs::Header NavPublisher::header() const
{
  std_msgs::Header h;
  h.stamp = stamp_;
  h.frame_id = frame_id_;
  return h;
}

void NavPublisher::handle_daheading(const ubx::Frame& frame)
{
  ubx::NavDAHeading m;
  const ubx::ParseResult result = ubx::parse_nav_daheading(frame.payload, &m);
  if (result == ubx::ParseResult::kUnsupportedVersion)
  {
    ROS_WARN_THROTTLE(30.0, "NAV-DAHEADING payload version 0x%02x not supported (expected 0x02)",
                      frame.payload.empty() ? 0 : frame.payload[0]);
    return;
  }
  if (result != ubx::ParseResult::kOk)
  {
    ROS_WARN_THROTTLE(30.0, "NAV-DAHEADING payload too short (%zu bytes)", frame.payload.size());
    return;
  }
  ublox_x20d_msgs::NavDAHeading msg;
  msg.header = header();
  msg.version = m.version;
  msg.itow = m.itow;
  msg.rel_pos_n = m.rel_pos_n;
  msg.rel_pos_e = m.rel_pos_e;
  msg.rel_pos_d = m.rel_pos_d;
  msg.rel_pos_length = m.rel_pos_length;
  msg.rel_pos_heading = m.rel_pos_heading;
  msg.acc_n = m.acc_n;
  msg.acc_e = m.acc_e;
  msg.acc_d = m.acc_d;
  msg.acc_length = m.acc_length;
  msg.acc_heading = m.acc_heading;
  msg.flags = m.flags;
  msg.gnss_fix_ok = m.gnss_fix_ok;
  msg.diff_soln = m.diff_soln;
  msg.rel_pos_valid = m.rel_pos_valid;
  msg.carr_soln = m.carr_soln;
  msg.rel_pos_heading_valid = m.rel_pos_heading_valid;
  daheading_pub_.publish(msg);

  std::lock_guard<std::mutex> lock(mutex_);
  last_daheading_ = m;
  have_daheading_ = true;
  last_daheading_steady_ = std::chrono::steady_clock::now();
  ++daheading_count_;
}

void NavPublisher::handle_pvt(const ubx::Frame& frame)
{
  ubx::NavPvt m;
  if (ubx::parse_nav_pvt(frame.payload, &m) != ubx::ParseResult::kOk)
  {
    ROS_WARN_THROTTLE(30.0, "NAV-PVT payload too short (%zu bytes)", frame.payload.size());
    return;
  }
  if (frame.payload.size() != ubx::kNavPvtLength && !pvt_length_warned_)
  {
    ROS_WARN("NAV-PVT payload is %zu bytes, expected %zu; extra bytes ignored", frame.payload.size(),
             ubx::kNavPvtLength);
    pvt_length_warned_ = true;
  }

  ublox_x20d_msgs::NavPVT msg;
  msg.header = header();
  msg.itow = m.itow;
  msg.year = m.year;
  msg.month = m.month;
  msg.day = m.day;
  msg.hour = m.hour;
  msg.min = m.min;
  msg.sec = m.sec;
  msg.nano = m.nano;
  msg.t_acc = m.t_acc;
  msg.valid_date = m.valid_date;
  msg.valid_time = m.valid_time;
  msg.fully_resolved = m.fully_resolved;
  msg.fix_type = m.fix_type;
  msg.gnss_fix_ok = m.gnss_fix_ok;
  msg.diff_soln = m.diff_soln;
  msg.carr_soln = m.carr_soln;
  msg.num_sv = m.num_sv;
  msg.lon = m.lon;
  msg.lat = m.lat;
  msg.height = m.height;
  msg.hmsl = m.hmsl;
  msg.h_acc = m.h_acc;
  msg.v_acc = m.v_acc;
  msg.invalid_llh = m.invalid_llh;
  msg.vel_n = m.vel_n;
  msg.vel_e = m.vel_e;
  msg.vel_d = m.vel_d;
  msg.g_speed = m.g_speed;
  msg.head_mot = m.head_mot;
  msg.s_acc = m.s_acc;
  msg.head_acc = m.head_acc;
  msg.p_dop = m.p_dop;
  pvt_pub_.publish(msg);

  if (nav::nav_sat_status(m) != nav::kStatusNoFix)
  {
    geometry_msgs::TwistWithCovarianceStamped vel;
    vel.header = header();
    const nav::Vector3 v = nav::enu_velocity(m);
    vel.twist.twist.linear.x = v.x;
    vel.twist.twist.linear.y = v.y;
    vel.twist.twist.linear.z = v.z;
    const auto cov = nav::twist_covariance(m);
    std::copy(cov.begin(), cov.end(), vel.twist.covariance.begin());
    vel_pub_.publish(vel);
  }

  std::string gga;
  if (nmea_pub_ && nmea::build_gga(m, &gga))
  {
    nmea_msgs::Sentence sentence;
    sentence.header = header();
    sentence.sentence = gga.substr(0, gga.size() - 2);  // without CRLF, as nmea_msgs carries it
    nmea_pub_.publish(sentence);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_pvt_ = m;
    have_pvt_ = true;
  }
  publish_fix_if_paired();
}

void NavPublisher::handle_hpposllh(const ubx::Frame& frame)
{
  ubx::NavHPPosLLH m;
  if (ubx::parse_nav_hpposllh(frame.payload, &m) != ubx::ParseResult::kOk)
  {
    ROS_WARN_THROTTLE(30.0, "NAV-HPPOSLLH payload too short (%zu bytes)", frame.payload.size());
    return;
  }
  if (m.version != ubx::kNavHPPosLLHVersion && !hpposllh_version_warned_)
  {
    ROS_WARN("NAV-HPPOSLLH payload version 0x%02x, layout of version 0x%02x assumed", m.version,
             ubx::kNavHPPosLLHVersion);
    hpposllh_version_warned_ = true;
  }

  ublox_x20d_msgs::NavHPPosLLH msg;
  msg.header = header();
  msg.version = m.version;
  msg.itow = m.itow;
  msg.lon = m.lon;
  msg.lat = m.lat;
  msg.height = m.height;
  msg.hmsl = m.hmsl;
  msg.lon_hp = m.lon_hp;
  msg.lat_hp = m.lat_hp;
  msg.height_hp = m.height_hp;
  msg.hmsl_hp = m.hmsl_hp;
  msg.h_acc = m.h_acc;
  msg.v_acc = m.v_acc;
  msg.invalid_llh = m.invalid_llh;
  hpposllh_pub_.publish(msg);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_hpposllh_ = m;
    have_hpposllh_ = true;
  }
  publish_fix_if_paired();
}

// NavSatFix needs the position from NAV-HPPOSLLH and the fix status from NAV-PVT of
// the same epoch; the receiver does not guarantee their order, so pair them by iTOW.
void NavPublisher::publish_fix_if_paired()
{
  ubx::NavPvt pvt;
  ubx::NavHPPosLLH hp;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_pvt_ || !have_hpposllh_ || last_pvt_.itow != last_hpposllh_.itow ||
        last_hpposllh_.itow == last_fix_itow_)
    {
      return;
    }
    pvt = last_pvt_;
    hp = last_hpposllh_;
    last_fix_itow_ = hp.itow;
  }

  sensor_msgs::NavSatFix fix;
  fix.header = header();
  fix.status.status = hp.invalid_llh ? nav::kStatusNoFix : nav::nav_sat_status(pvt);
  fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  fix.latitude = nav::latitude_deg(hp);
  fix.longitude = nav::longitude_deg(hp);
  fix.altitude = nav::ellipsoid_height_m(hp);
  const auto cov = nav::position_covariance(hp);
  std::copy(cov.begin(), cov.end(), fix.position_covariance.begin());
  fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix_pub_.publish(fix);
}

}  // namespace ublox_x20d_driver
