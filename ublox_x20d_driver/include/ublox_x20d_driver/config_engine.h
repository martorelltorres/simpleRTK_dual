// Copyright 2026 Monkey Robotics
// Modifications Copyright 2026 Antoni Martorell
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

//
// Adapted from MonKey-Robotics/ublox_zedx20d,
// ublox_dgnss_node/include/ublox_dgnss_node/config_engine.hpp.
// Changes: ROS 1 package layout; device-parameter sweep, runtime key enqueueing and
// service-requested resets removed; MON-VER compatibility check added (INCOMPATIBLE).

#ifndef UBLOX_X20D_DRIVER_CONFIG_ENGINE_H
#define UBLOX_X20D_DRIVER_CONFIG_ENGINE_H

// Configuration apply/verify state machine for the u-blox driver.
//
// Pure logic: no ROS, no I/O. The node feeds it events (ACK/NAK, CFG-VALGET replies,
// MON-VER, NAV frames, NMEA sentences, serial port loss and reopen) and calls tick() from a timer; tick()
// returns the actions the node must execute (send a CFG-VALSET, send a verification
// CFG-VALGET, send UBX-CFG-RST, mark keys, log).
//
// Why it exists: on a cold start the ZED-X20D (HDG 2.00) acknowledges a CFG-VALSET and
// does not apply it. The old fire-and-forget startup path never noticed, so the receiver
// kept its default NMEA output and never enabled the UBX NAV messages. This engine sends
// one frame at a time, verifies every user key by reading it back from the RAM layer and
// escalates through a ladder when the readback disagrees:
//   rung a  transactionless CFG-VALSET, retried with backoff
//   rung b  the same keys as a configuration transaction (start ... apply)
//   rung c  UBX-CFG-RST controlled software reset (hot start), then re-apply
//   rung d  never give up: one attempt every degraded_retry_s at ERROR level

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ublox_x20d_driver
{
namespace cfgeng
{

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using KeyId = uint32_t;

struct KeyInfo
{
  KeyId key;
  std::string name;
};

struct KeyBytes
{
  KeyId key;
  std::vector<uint8_t> bytes;   // packed value, storage_size() bytes
};

struct Params
{
  double handshake_timeout_s = 5.0;      // MON-VER wait before sending configuration anyway
  double handshake_repoll_s = 1.0;       // MON-VER re-poll period
  double ack_timeout_s = 1.5;            // CFG-VALSET ACK wait (u-blox promises < 1 s)
  double verify_timeout_s = 1.5;         // verification CFG-VALGET reply wait
  int valset_attempts = 4;               // rung a attempts
  double valset_backoff_s = 0.5;         // rung a base backoff (0.5/1/2/4 s)
  int txn_attempts = 2;                  // rung b attempts (0 disables rung b)
  int reset_attempts = 3;                // rung c resets (0 disables CFG-RST)
  uint8_t reset_mode = 0x01;             // UBX-CFG-RST resetMode for rung c
  double reset_settle_s = 3.0;           // wait for the serial port to drop after CFG-RST
  double degraded_retry_s = 30.0;        // rung d cadence
  bool watchdog_enabled = true;
  double nav_watchdog_s = 5.0;           // no UBX-NAV for this long => re-verify
  // tolerated NMEA rate with NMEA disabled: $GNTHS leaks at exactly 1/s on HDG 2.00 (jitter
  // makes a 5 s window count 6), the unapplied default set is ~46/s
  double nmea_watchdog_per_s = 3.0;
  double watchdog_min_interval_s = 30.0;  // minimum gap between two watchdog trips
  double nmea_summary_period_s = 5.0;    // INFO summary period for NMEA traffic
};

enum class State
{
  IDLE,               // not started (port not open yet, or lost)
  HANDSHAKE,          // waiting for MON-VER
  APPLY_SEND,         // next CFG-VALSET chunk goes out on the next tick
  APPLY_WAIT_ACK,     // transactionless frame in flight
  TXN_START_WAIT_ACK,  // transaction start/ongoing frame in flight
  TXN_END_WAIT_ACK,   // transaction apply frame in flight
  VERIFY_SEND,        // next verification CFG-VALGET goes out on the next tick
  VERIFY_WAIT,        // verification CFG-VALGET in flight
  APPLY_BACKOFF,      // waiting before the next attempt
  RESET_SETTLE,       // UBX-CFG-RST sent, waiting for re-enumeration or settle
  READY,              // configuration verified; watchdog armed
  DEGRADED,           // rung d
  INCOMPATIBLE        // MON-VER reports another module or firmware; never configured
};

enum class Rung { A, B, C, D };

enum class LogLevel { DEBUG, INFO, WARN, ERROR };

enum class ActionType
{
  POLL_MON_VER,
  SEND_VALSET,          // keys + txn_action (0 transactionless, 1 start, 2 ongoing, 3 apply)
  SEND_VALGET_VERIFY,   // keys, RAM layer
  SEND_CFG_RST,         // hot start, reset_mode
  MARK_KEYS_VERIFIED,
  MARK_KEYS_ACKNAK,
  LOG
};

struct Action
{
  ActionType type;
  std::vector<KeyId> keys;
  uint8_t txn_action = 0;
  LogLevel level = LogLevel::INFO;
  std::string text;
};

struct StartInfo
{
  std::vector<KeyInfo> user_keys;
  bool nav_msgout_enabled = false;   // some CFG_MSGOUT_UBX_NAV_*_USB user key is > 0
  bool nmea_disabled = false;        // CFG_USBOUTPROT_NMEA=false is a user key
  double nav_period_s = 1.0;         // CFG_RATE_MEAS * CFG_RATE_NAV
};

inline const char * to_string(State s)
{
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::HANDSHAKE: return "HANDSHAKE";
    case State::APPLY_SEND: return "APPLY_SEND";
    case State::APPLY_WAIT_ACK: return "APPLY_WAIT_ACK";
    case State::TXN_START_WAIT_ACK: return "TXN_START_WAIT_ACK";
    case State::TXN_END_WAIT_ACK: return "TXN_END_WAIT_ACK";
    case State::VERIFY_SEND: return "VERIFY_SEND";
    case State::VERIFY_WAIT: return "VERIFY_WAIT";
    case State::APPLY_BACKOFF: return "APPLY_BACKOFF";
    case State::RESET_SETTLE: return "RESET_SETTLE";
    case State::READY: return "READY";
    case State::DEGRADED: return "DEGRADED";
    case State::INCOMPATIBLE: return "INCOMPATIBLE";
    default: return "?";
  }
}

inline const char * to_string(Rung r)
{
  switch (r) {
    case Rung::A: return "rung a";
    case Rung::B: return "rung b";
    case Rung::C: return "rung c";
    case Rung::D: return "rung d";
    default: return "?";
  }
}

class ConfigEngine
{
public:
  static constexpr size_t kMaxKeysPerFrame = 64;
  static constexpr uint8_t kClsCfg = 0x06;
  static constexpr uint8_t kIdValSet = 0x8a;
  static constexpr uint8_t kIdValGet = 0x8b;

  explicit ConfigEngine(const Params & p = Params())
  : p_(p) {}

  void set_params(const Params & p) {p_ = p;}
  const Params & params() const {return p_;}

  // ---- events (call under the node's engine mutex; never call out while holding it) ----

  // Serial port opened (first connection or reopen after a loss): snapshot of the user keys.
  void start(const StartInfo & info, TimePoint now)
  {
    started_ = true;
    user_keys_.clear();
    names_.clear();
    for (const auto & k : info.user_keys) {
      if (dropped_.count(k.key)) {continue;}
      add_key(k);
    }
    nav_msgout_enabled_ = info.nav_msgout_enabled;
    nmea_disabled_ = info.nmea_disabled;
    nav_period_s_ = info.nav_period_s > 0.0 ? info.nav_period_s : 1.0;
    clear_inflight();
    if (reset_in_progress_ && reset_count_ > 0) {
      log(
        LogLevel::WARN, "serial port reopened after UBX-CFG-RST #" +
        std::to_string(reset_count_) + "; re-applying configuration (" +
        std::string(to_string(rung_)) + ")");
    } else if (reset_in_progress_) {
      log(
        LogLevel::WARN,
        "serial port reopened after the requested reset; re-applying configuration");
    } else {
      rung_ = Rung::A;
      attempt_ = 0;
      reset_count_ = 0;
      isolate_ = false;
      watchdog_episode_ = false;
    }
    set_job(all_user_keys());
    enter_handshake(now);
  }

  // MON-VER reply. `compatible` is the node's verdict on module and firmware; `summary`
  // is logged. An incompatible receiver is never configured.
  void on_mon_ver(bool compatible, const std::string & summary, TimePoint now)
  {
    (void)now;
    if (state_ != State::HANDSHAKE) {return;}
    if (!compatible) {
      log(
        LogLevel::ERROR, "receiver MON-VER: " + summary +
        "; not a ZED-X20D with HDG 2.00 or later firmware, configuration disabled");
      clear_inflight();
      state_ = State::INCOMPATIBLE;
      return;
    }
    log(LogLevel::INFO, "receiver MON-VER: " + summary);
    mon_ver_seen_ = true;
  }

  void on_ack(uint8_t cls, uint8_t id, TimePoint now)
  {
    if (cls != kClsCfg || id != kIdValSet) {return;}
    if (!nominal()) {
      log(
        LogLevel::INFO, "ACK-ACK CFG-VALSET after " + ms_since(sent_at_, now) + " ms (" +
        std::string(to_string(rung_)) + ", attempt " + std::to_string(attempt_) + ", " +
        to_string(state_) + ")");
    }
    switch (state_) {
      case State::APPLY_WAIT_ACK:
        chunk_idx_++;
        if (chunk_idx_ < chunks_.size()) {
          state_ = State::APPLY_SEND;
        } else {
          begin_verify(now);
        }
        break;
      case State::TXN_START_WAIT_ACK:
        chunk_idx_++;
        if (chunk_idx_ < chunks_.size()) {
          state_ = State::APPLY_SEND;
        } else {
          // apply-and-end frame: repeat the last chunk (cfgData must not be empty)
          Action a;
          a.type = ActionType::SEND_VALSET;
          a.keys = chunks_.back();
          a.txn_action = 3;
          pending_.push_back(a);
          inflight_keys_ = chunks_.back();
          sent_at_ = now;
          state_ = State::TXN_END_WAIT_ACK;
        }
        break;
      case State::TXN_END_WAIT_ACK:
        begin_verify(now);
        break;
      default:
        break;   // late or unrelated ACK
    }
  }

  void on_nak(uint8_t cls, uint8_t id, TimePoint now)
  {
    if (cls != kClsCfg) {return;}
    if (id == kIdValGet) {
      if (state_ == State::VERIFY_WAIT) {
        fail("verification CFG-VALGET NAKed", now);
      }
      return;
    }
    if (id != kIdValSet) {return;}
    if (state_ != State::APPLY_WAIT_ACK && state_ != State::TXN_START_WAIT_ACK &&
      state_ != State::TXN_END_WAIT_ACK)
    {
      return;
    }
    if (!nominal()) {
      log(
        LogLevel::INFO, "ACK-NAK CFG-VALSET after " + ms_since(sent_at_, now) + " ms (" +
        std::string(to_string(rung_)) + ", attempt " + std::to_string(attempt_) + ")");
    }
    if (isolate_ && inflight_keys_.size() == 1) {
      // A key NAKed on its own is a bad value, not a receiver state: drop it.
      const KeyId k = inflight_keys_.front();
      log(
        LogLevel::ERROR, "receiver NAKed " + name_of(k) +
        " on its own; dropping it from the configuration (PARAM_ACKNAK)");
      Action a;
      a.type = ActionType::MARK_KEYS_ACKNAK;
      a.keys = {k};
      pending_.push_back(a);
      drop_key(k);
      // keep the position: chunks_ was rebuilt without the key
      if (chunk_idx_ < chunks_.size()) {
        state_ = State::APPLY_SEND;
      } else if (!job_keys_.empty()) {
        begin_verify(now);
      } else {
        on_verified(now);
      }
      return;
    }
    if (rung_ == Rung::A && attempt_ >= 2 && inflight_keys_.size() > 1 && !isolate_) {
      isolate_ = true;
      log(LogLevel::WARN, "CFG-VALSET NAKed again; isolating keys one per frame");
      set_job(job_keys_);
      state_ = State::APPLY_SEND;
      return;
    }
    fail("CFG-VALSET NAKed", now);
  }

  // The node built and wrote the frame: these are the exact wire bytes it carried.
  void on_valset_sent(const std::vector<KeyBytes> & expected, TimePoint now)
  {
    for (const auto & kb : expected) {
      expected_[kb.key] = kb.bytes;
    }
    sent_at_ = now;
  }

  void on_send_failed(const std::string & what, TimePoint now)
  {
    if (state_ == State::APPLY_WAIT_ACK || state_ == State::TXN_START_WAIT_ACK ||
      state_ == State::TXN_END_WAIT_ACK || state_ == State::VERIFY_WAIT)
    {
      fail("serial write failed: " + what, now);
    }
  }

  // Returns true when the reply answered the outstanding verification request (the node
  // must then NOT feed it to the generic parameter path).
  bool on_valget_response(const std::vector<KeyBytes> & pairs, TimePoint now)
  {
    if (state_ != State::VERIFY_WAIT) {return false;}
    std::set<KeyId> got;
    for (const auto & kb : pairs) {
      got.insert(kb.key);
    }
    if (got != verify_set_) {return false;}
    std::string mismatches;
    for (const auto & kb : pairs) {
      auto it = expected_.find(kb.key);
      if (it == expected_.end()) {continue;}
      if (!same_bytes(it->second, kb.bytes)) {
        if (!mismatches.empty()) {mismatches += ", ";}
        mismatches += name_of(kb.key) + " expected " + hex(it->second) + " got " + hex(kb.bytes);
      }
    }
    if (!mismatches.empty()) {
      fail("readback mismatch: " + mismatches, now);
      return true;
    }
    verify_chunk_idx_++;
    if (verify_chunk_idx_ < verify_chunks_.size()) {
      state_ = State::VERIFY_SEND;
    } else {
      on_verified(now);
    }
    return true;
  }

  void on_detached(TimePoint now)
  {
    (void)now;
    if (state_ == State::RESET_SETTLE && reset_in_progress_) {
      log(LogLevel::INFO, "serial port lost after UBX-CFG-RST (expected; waiting for reopen)");
    }
    started_ = false;
    state_ = State::IDLE;
    clear_inflight();
  }

  void on_nav_frame(TimePoint now)
  {
    last_nav_ = now;
    have_last_nav_ = true;
  }

  void on_nmea(size_t sentences, const std::string & last, TimePoint now)
  {
    (void)now;
    nmea_count_ += sentences;
    if (!last.empty()) {nmea_last_ = last;}
  }

  // ---- periodic ----

  std::vector<Action> tick(TimePoint now)
  {
    std::vector<Action> out;
    out.swap(pending_);
    nmea_summary(now, out);
    if (!started_) {
      return out;
    }
    switch (state_) {
      case State::IDLE:
        break;
      case State::HANDSHAKE:
        if (mon_ver_seen_) {
          to_apply(now, out);
        } else if (seconds(now - state_since_) >= p_.handshake_timeout_s) {
          log(
            LogLevel::WARN, "no MON-VER reply within " + fmt(p_.handshake_timeout_s) +
            " s; sending configuration anyway", out);
          to_apply(now, out);
        } else if (seconds(now - handshake_last_poll_) >= p_.handshake_repoll_s) {
          out.push_back(Action{ActionType::POLL_MON_VER, {}, 0, LogLevel::INFO, ""});
          handshake_last_poll_ = now;
        }
        break;
      case State::APPLY_SEND:
        emit_next_chunk(now, out);
        break;
      case State::APPLY_WAIT_ACK:
      case State::TXN_START_WAIT_ACK:
      case State::TXN_END_WAIT_ACK:
        if (seconds(now - sent_at_) >= p_.ack_timeout_s) {
          fail("no ACK for CFG-VALSET within " + fmt(p_.ack_timeout_s) + " s", now, &out);
        }
        break;
      case State::VERIFY_SEND:
        emit_verify_chunk(now, out);
        break;
      case State::VERIFY_WAIT:
        if (seconds(now - verify_sent_at_) >= p_.verify_timeout_s) {
          fail(
            "no CFG-VALGET reply for verification within " + fmt(p_.verify_timeout_s) + " s",
            now, &out);
        }
        break;
      case State::APPLY_BACKOFF:
        if (now >= backoff_until_) {
          state_ = State::APPLY_SEND;
          emit_next_chunk(now, out);
        }
        break;
      case State::RESET_SETTLE:
        if (seconds(now - reset_sent_at_) >= p_.reset_settle_s) {
          log(
            LogLevel::WARN, "no serial port loss within " + fmt(p_.reset_settle_s) +
            " s of the reset; re-handshaking on the same connection", out);
          clear_inflight();
          set_job(all_user_keys());
          enter_handshake(now, &out);
        }
        break;
      case State::READY:
        watchdog(now, out);
        break;
      case State::DEGRADED:
        if (seconds(now - degraded_last_attempt_) >= p_.degraded_retry_s) {
          std::vector<KeyId> keys = all_user_keys();
          degraded_last_attempt_ = now;
          log(
            LogLevel::ERROR, "receiver still refuses configuration (" +
            fmt(seconds(now - degraded_since_)) + " s in rung d); trying once more", out);
          rung_ = Rung::D;
          attempt_ = 0;
          isolate_ = false;
          set_job(keys);
          state_ = State::APPLY_SEND;
          emit_next_chunk(now, out);
        }
        break;
      default:
        break;
    }
    return out;
  }

  // ---- introspection ----
  State state() const {return state_;}
  Rung rung() const {return rung_;}
  int attempt() const {return attempt_;}
  int reset_count() const {return reset_count_;}
  bool started() const {return started_;}
  bool reset_in_progress() const {return reset_in_progress_;}
  bool nominal() const
  {
    return rung_ == Rung::A && attempt_ <= 1 && !watchdog_episode_ && !reset_in_progress_;
  }
  size_t user_key_count() const {return user_keys_.size();}

private:
  // ---- helpers ----
  static double seconds(Clock::duration d)
  {
    return std::chrono::duration<double>(d).count();
  }
  static std::string fmt(double v)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f", v);
    return buf;
  }
  static std::string ms_since(TimePoint from, TimePoint now)
  {
    return std::to_string(
      std::chrono::duration_cast<std::chrono::milliseconds>(now - from).count());
  }
  static std::string hex(const std::vector<uint8_t> & b)
  {
    std::string s = "0x";
    char buf[4];
    for (auto v : b) {
      std::snprintf(buf, sizeof(buf), "%02x", v);
      s += buf;
    }
    return s;
  }
  static bool same_bytes(const std::vector<uint8_t> & a, const std::vector<uint8_t> & b)
  {
    const size_t n = std::min(a.size(), b.size());
    if (n == 0) {return a.size() == b.size();}
    return std::equal(a.begin(), a.begin() + n, b.begin());
  }
  std::string name_of(KeyId k) const
  {
    auto it = names_.find(k);
    if (it != names_.end()) {return it->second;}
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%08x", k);
    return buf;
  }
  void log(LogLevel level, const std::string & text)
  {
    pending_.push_back(Action{ActionType::LOG, {}, 0, level, text});
  }
  void log(LogLevel level, const std::string & text, std::vector<Action> & out)
  {
    out.push_back(Action{ActionType::LOG, {}, 0, level, text});
  }
  void add_key(const KeyInfo & k)
  {
    names_[k.key] = k.name;
    if (std::find(user_keys_.begin(), user_keys_.end(), k.key) == user_keys_.end()) {
      user_keys_.push_back(k.key);
    }
  }
  void drop_key(KeyId k)
  {
    dropped_.insert(k);
    user_keys_.erase(std::remove(user_keys_.begin(), user_keys_.end(), k), user_keys_.end());
    job_keys_.erase(std::remove(job_keys_.begin(), job_keys_.end(), k), job_keys_.end());
    expected_.erase(k);
    // rebuild the chunks from the current position
    std::vector<KeyId> remaining;
    for (size_t i = chunk_idx_ + 1; i < chunks_.size(); i++) {
      for (auto kk : chunks_[i]) {
        if (kk != k) {remaining.push_back(kk);}
      }
    }
    std::vector<std::vector<KeyId>> done(chunks_.begin(), chunks_.begin() + chunk_idx_);
    chunks_ = done;
    for (auto & c : split(remaining)) {chunks_.push_back(c);}
  }
  std::vector<KeyId> all_user_keys() const {return user_keys_;}
  std::vector<std::vector<KeyId>> split(const std::vector<KeyId> & keys) const
  {
    std::vector<std::vector<KeyId>> chunks;
    const size_t n = isolate_ ? 1 : kMaxKeysPerFrame;
    for (size_t i = 0; i < keys.size(); i += n) {
      chunks.emplace_back(keys.begin() + i, keys.begin() + std::min(keys.size(), i + n));
    }
    return chunks;
  }
  void set_job(const std::vector<KeyId> & keys_in)
  {
    const std::vector<KeyId> keys = keys_in;   // may alias job_keys_
    job_keys_.clear();
    for (auto k : keys) {
      if (dropped_.count(k)) {continue;}
      if (std::find(job_keys_.begin(), job_keys_.end(), k) == job_keys_.end()) {
        job_keys_.push_back(k);
      }
    }
    chunks_ = split(job_keys_);
    chunk_idx_ = 0;
  }
  void clear_inflight()
  {
    inflight_keys_.clear();
    verify_set_.clear();
    verify_chunks_.clear();
    verify_chunk_idx_ = 0;
  }
  void enter_handshake(TimePoint now, std::vector<Action> * out = nullptr)
  {
    state_ = State::HANDSHAKE;
    state_since_ = now;
    mon_ver_seen_ = false;
    handshake_last_poll_ = now;
    Action a{ActionType::POLL_MON_VER, {}, 0, LogLevel::INFO, ""};
    if (out) {
      out->push_back(a);
    } else {
      pending_.push_back(a);
    }
  }
  void to_apply(TimePoint now, std::vector<Action> & out)
  {
    if (job_keys_.empty()) {
      log(LogLevel::INFO, "no user configuration keys to apply", out);
      on_verified(now, &out);
      return;
    }
    chunk_idx_ = 0;
    state_ = State::APPLY_SEND;
    emit_next_chunk(now, out);
  }
  void emit_next_chunk(TimePoint now, std::vector<Action> & out)
  {
    if (chunk_idx_ == 0) {
      attempt_++;
    }
    if (chunk_idx_ >= chunks_.size()) {
      begin_verify(now);
      emit_verify_chunk(now, out);
      return;
    }
    Action a;
    a.type = ActionType::SEND_VALSET;
    a.keys = chunks_[chunk_idx_];
    if (rung_ == Rung::B) {
      a.txn_action = (chunk_idx_ == 0) ? 1 : 2;
      state_ = State::TXN_START_WAIT_ACK;
    } else {
      a.txn_action = 0;
      state_ = State::APPLY_WAIT_ACK;
    }
    inflight_keys_ = a.keys;
    sent_at_ = now;
    out.push_back(a);
  }
  void begin_verify(TimePoint now)
  {
    (void)now;
    verify_chunks_.clear();
    const size_t n = kMaxKeysPerFrame;
    for (size_t i = 0; i < job_keys_.size(); i += n) {
      verify_chunks_.emplace_back(
        job_keys_.begin() + i, job_keys_.begin() + std::min(job_keys_.size(), i + n));
    }
    verify_chunk_idx_ = 0;
    state_ = State::VERIFY_SEND;
  }
  void emit_verify_chunk(TimePoint now, std::vector<Action> & out)
  {
    if (verify_chunk_idx_ >= verify_chunks_.size()) {
      on_verified(now, &out);
      return;
    }
    Action a;
    a.type = ActionType::SEND_VALGET_VERIFY;
    a.keys = verify_chunks_[verify_chunk_idx_];
    verify_set_ = std::set<KeyId>(a.keys.begin(), a.keys.end());
    verify_sent_at_ = now;
    state_ = State::VERIFY_WAIT;
    out.push_back(a);
  }
  void on_verified(TimePoint now, std::vector<Action> * out = nullptr)
  {
    std::vector<Action> & sink = out ? *out : pending_;
    if (!job_keys_.empty()) {
      Action m;
      m.type = ActionType::MARK_KEYS_VERIFIED;
      m.keys = job_keys_;
      sink.push_back(m);
    }
    std::string how;
    if (state_ == State::DEGRADED || rung_ == Rung::D) {
      how = " (recovered after " + fmt(seconds(now - degraded_since_)) + " s in rung d)";
    } else if (reset_in_progress_) {
      how = " (after " + std::to_string(reset_count_) + " reset(s), " + to_string(rung_) + ")";
    } else if (!nominal()) {
      how = " (" + std::string(to_string(rung_)) + ", attempt " + std::to_string(attempt_) + ")";
    }
    log(
      LogLevel::INFO, "user configuration verified on device (" +
      std::to_string(job_keys_.size()) + " keys)" + how, sink);
    rung_ = Rung::A;
    attempt_ = 0;
    reset_count_ = 0;
    reset_in_progress_ = false;
    watchdog_episode_ = false;
    isolate_ = false;
    clear_inflight();
    state_ = State::READY;
    ready_since_ = now;
    last_nav_ = now;
    have_last_nav_ = true;
  }
  void fail(const std::string & reason, TimePoint now, std::vector<Action> * out = nullptr)
  {
    std::vector<Action> & sink = out ? *out : pending_;
    const int budget = (rung_ == Rung::A) ? p_.valset_attempts :
      (rung_ == Rung::B) ? p_.txn_attempts : 1;
    log(
      LogLevel::WARN, "config NOT applied on device (" + std::string(to_string(rung_)) +
      ", attempt " + std::to_string(attempt_) + "/" + std::to_string(budget) + "): " + reason,
      sink);
    clear_inflight();
    chunk_idx_ = 0;
    switch (rung_) {
      case Rung::A:
        if (attempt_ < p_.valset_attempts) {
          const int exp = std::max(attempt_ - 1, 0);
          backoff_until_ = now + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(p_.valset_backoff_s * (1 << exp)));
          state_ = State::APPLY_BACKOFF;
          return;
        }
        if (p_.txn_attempts > 0) {
          log(LogLevel::WARN, "rung a exhausted; trying the transaction form (rung b)", sink);
          rung_ = Rung::B;
          attempt_ = 0;
          isolate_ = false;
          set_job(job_keys_);
          state_ = State::APPLY_SEND;
          return;
        }
        escalate_reset(sink, now);
        return;
      case Rung::B:
        if (attempt_ < p_.txn_attempts) {
          backoff_until_ = now + std::chrono::seconds(1);
          state_ = State::APPLY_BACKOFF;
          return;
        }
        escalate_reset(sink, now);
        return;
      case Rung::C:
        escalate_reset(sink, now);
        return;
      case Rung::D:
      default:
        state_ = State::DEGRADED;
        degraded_last_attempt_ = now;
        return;
    }
  }
  void escalate_reset(std::vector<Action> & sink, TimePoint now)
  {
    if (reset_count_ < p_.reset_attempts) {
      reset_count_++;
      char mode[8];
      std::snprintf(mode, sizeof(mode), "0x%02x", p_.reset_mode);
      log(
        LogLevel::WARN, "rung b exhausted; sending UBX-CFG-RST (hot start, resetMode " +
        std::string(mode) + "), reset " + std::to_string(reset_count_) + "/" +
        std::to_string(p_.reset_attempts) + " (rung c)", sink);
      sink.push_back(Action{ActionType::SEND_CFG_RST, {}, 0, LogLevel::INFO, ""});
      rung_ = Rung::A;   // after the reset the ladder restarts; reset_count_ persists
      attempt_ = 0;
      isolate_ = false;
      reset_in_progress_ = true;
      state_ = State::RESET_SETTLE;
      reset_sent_at_ = now;
      return;
    }
    log(
      LogLevel::ERROR, "receiver refuses configuration after " +
      std::to_string(reset_count_) + " reset(s); retrying every " +
      fmt(p_.degraded_retry_s) + " s (rung d)", sink);
    rung_ = Rung::D;
    attempt_ = 0;
    reset_in_progress_ = false;
    state_ = State::DEGRADED;
    degraded_since_ = now;
    degraded_last_attempt_ = now;
  }
  void watchdog(TimePoint now, std::vector<Action> & out)
  {
    if (!p_.watchdog_enabled) {return;}
    if (last_trip_ != TimePoint() && seconds(now - last_trip_) < p_.watchdog_min_interval_s) {
      return;
    }
    std::string why;
    const double nav_window = std::max(p_.nav_watchdog_s, 3.0 * nav_period_s_);
    const bool nav_silent = nav_msgout_enabled_ && have_last_nav_ &&
      seconds(now - last_nav_) > nav_window;
    // Only a window that started after READY counts: before that the receiver may still
    // have been streaming its default NMEA set.
    const bool nmea_leaking = nmea_disabled_ && nmea_rate_window_start_ >= ready_since_ &&
      nmea_rate_ > p_.nmea_watchdog_per_s;
    if (nav_silent) {
      why = "no UBX-NAV message for " + fmt(seconds(now - last_nav_)) + " s";
    } else if (nmea_leaking) {
      why = "NMEA still streaming at " + fmt(nmea_rate_) + "/s with CFG_USBOUTPROT_NMEA=false";
    }
    if (why.empty()) {return;}
    log(LogLevel::WARN, why + "; re-verifying the configuration", out);
    last_trip_ = now;
    watchdog_episode_ = true;
    rung_ = Rung::A;
    attempt_ = 0;
    isolate_ = false;
    set_job(all_user_keys());
    // Keep the expected bytes from the last apply so the readback can be compared; the
    // node refreshes them on the next SEND_VALSET anyway.
    begin_verify(now);
    emit_verify_chunk(now, out);
  }
  void nmea_summary(TimePoint now, std::vector<Action> & out)
  {
    if (nmea_window_start_ == TimePoint()) {
      nmea_window_start_ = now;
      return;
    }
    const double elapsed = seconds(now - nmea_window_start_);
    if (elapsed < p_.nmea_summary_period_s) {return;}
    nmea_rate_ = elapsed > 0.0 ? static_cast<double>(nmea_count_) / elapsed : 0.0;
    nmea_rate_window_start_ = nmea_window_start_;
    if (nmea_count_ > 0) {
      log(
        LogLevel::INFO, "nmea: " + std::to_string(nmea_count_) + " sentences in " +
        fmt(elapsed) + " s, last: " + nmea_last_, out);
    }
    nmea_count_ = 0;
    nmea_window_start_ = now;
  }

  Params p_;
  State state_ = State::IDLE;
  Rung rung_ = Rung::A;
  int attempt_ = 0;
  int reset_count_ = 0;
  bool started_ = false;
  bool reset_in_progress_ = false;
  bool watchdog_episode_ = false;
  bool isolate_ = false;
  bool mon_ver_seen_ = false;

  std::vector<KeyId> user_keys_;
  std::map<KeyId, std::string> names_;
  std::set<KeyId> dropped_;
  bool nav_msgout_enabled_ = false;
  bool nmea_disabled_ = false;
  double nav_period_s_ = 1.0;

  std::vector<KeyId> job_keys_;
  std::vector<std::vector<KeyId>> chunks_;
  size_t chunk_idx_ = 0;

  std::vector<KeyId> inflight_keys_;
  std::map<KeyId, std::vector<uint8_t>> expected_;
  TimePoint sent_at_;

  std::vector<std::vector<KeyId>> verify_chunks_;
  size_t verify_chunk_idx_ = 0;
  std::set<KeyId> verify_set_;
  TimePoint verify_sent_at_;

  TimePoint state_since_;
  TimePoint handshake_last_poll_;
  TimePoint backoff_until_;
  TimePoint reset_sent_at_;
  TimePoint degraded_since_;
  TimePoint degraded_last_attempt_;

  TimePoint ready_since_;
  TimePoint last_nav_;
  bool have_last_nav_ = false;
  TimePoint last_trip_;

  size_t nmea_count_ = 0;
  std::string nmea_last_;
  TimePoint nmea_window_start_;
  TimePoint nmea_rate_window_start_;  // start of the window nmea_rate_ was measured over
  double nmea_rate_ = 0.0;

  std::vector<Action> pending_;
};

}  // namespace cfgeng
}  // namespace ublox_x20d_driver

#endif  // UBLOX_X20D_DRIVER_CONFIG_ENGINE_H
