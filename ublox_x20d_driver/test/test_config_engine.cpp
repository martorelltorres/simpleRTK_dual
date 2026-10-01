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
// Adapted from MonKey-Robotics/ublox_zedx20d, ublox_dgnss_node/test/test_config_engine.cpp.
// Changes: sweep, runtime key and service reset cases removed; MON-VER compatibility,
// allow_receiver_reset = false and serial port loss cases added.

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

#include "ublox_x20d_driver/config_engine.h"

using ublox_x20d_driver::cfgeng::Action;
using ublox_x20d_driver::cfgeng::ActionType;
using ublox_x20d_driver::cfgeng::ConfigEngine;
using ublox_x20d_driver::cfgeng::KeyBytes;
using ublox_x20d_driver::cfgeng::KeyId;
using ublox_x20d_driver::cfgeng::KeyInfo;
using ublox_x20d_driver::cfgeng::LogLevel;
using ublox_x20d_driver::cfgeng::Params;
using ublox_x20d_driver::cfgeng::Rung;
using ublox_x20d_driver::cfgeng::StartInfo;
using ublox_x20d_driver::cfgeng::State;
using ublox_x20d_driver::cfgeng::TimePoint;

namespace
{

const char kMonVer[] = "MOD=ZED-X20D FWVER=HDG 2.00 PROTVER=57.02";

constexpr KeyId K_NMEA = 0x10780002;      // CFG_USBOUTPROT_NMEA (L)
constexpr KeyId K_RATE = 0x30210001;      // CFG_RATE_MEAS (U2)
constexpr KeyId K_HPPOS = 0x20910036;     // CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB (U1)
constexpr KeyId K_OFFSET = 0x401100e4;    // CFG_NAVSPG_DAHEADING_OFFSET (I4)

struct Fixture
{
  ConfigEngine engine;
  TimePoint now;
  std::vector<Action> last;

  explicit Fixture(Params p = Params())
  : engine(p), now(std::chrono::steady_clock::time_point() + std::chrono::seconds(1000)) {}

  StartInfo info() const
  {
    StartInfo i;
    i.user_keys = {
      {K_NMEA, "CFG_USBOUTPROT_NMEA"}, {K_RATE, "CFG_RATE_MEAS"},
      {K_HPPOS, "CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB"}, {K_OFFSET, "CFG_NAVSPG_DAHEADING_OFFSET"}};
    i.nav_msgout_enabled = true;
    i.nmea_disabled = true;
    i.nav_period_s = 1.0;
    return i;
  }

  void advance(double s)
  {
    now += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(s));
  }

  std::vector<Action> tick()
  {
    last = engine.tick(now);
    return last;
  }

  // ticks until an action of the given type shows up (or gives up after n ticks)
  bool tick_until(ActionType t, int n = 200, double step = 0.05)
  {
    for (int i = 0; i < n; i++) {
      for (const auto & a : tick()) {
        if (a.type == t) {return true;}
      }
      advance(step);
    }
    return false;
  }

  static int count(const std::vector<Action> & v, ActionType t)
  {
    int c = 0;
    for (const auto & a : v) {
      if (a.type == t) {c++;}
    }
    return c;
  }

  static const Action * find(const std::vector<Action> & v, ActionType t)
  {
    for (const auto & a : v) {
      if (a.type == t) {return &a;}
    }
    return nullptr;
  }

  static bool has_log(const std::vector<Action> & v, LogLevel level, const std::string & needle)
  {
    for (const auto & a : v) {
      if (a.type == ActionType::LOG && a.level == level &&
        a.text.find(needle) != std::string::npos)
      {
        return true;
      }
    }
    return false;
  }

  std::vector<KeyBytes> expected_for(const std::vector<KeyId> & keys) const
  {
    std::vector<KeyBytes> e;
    for (auto k : keys) {
      e.push_back(KeyBytes{k, value_of(k)});
    }
    return e;
  }

  static std::vector<uint8_t> value_of(KeyId k)
  {
    switch (k) {
      case K_NMEA: return {0x00};
      case K_RATE: return {0xE8, 0x03};
      case K_HPPOS: return {0x01};
      case K_OFFSET: return {0xD8, 0xDC, 0xFF, 0xFF};
      default: return {0x00};
    }
  }

  static std::vector<uint8_t> default_of(KeyId k)
  {
    switch (k) {
      case K_NMEA: return {0x01};
      case K_RATE: return {0xE8, 0x03};
      case K_HPPOS: return {0x00};
      case K_OFFSET: return {0x00, 0x00, 0x00, 0x00};
      default: return {0x00};
    }
  }

  // drive: start -> handshake -> first VALSET out; returns the VALSET action
  Action start_to_valset()
  {
    engine.start(info(), now);
    auto acts = tick();
    EXPECT_TRUE(find(acts, ActionType::POLL_MON_VER) != nullptr);
    EXPECT_EQ(engine.state(), State::HANDSHAKE);
    engine.on_mon_ver(true, kMonVer, now);
    acts = tick();
    const Action * vs = find(acts, ActionType::SEND_VALSET);
    EXPECT_TRUE(vs != nullptr);
    if (!vs) {return Action{};}
    engine.on_valset_sent(expected_for(vs->keys), now);
    return *vs;
  }

  // the node ACKs the frame; engine asks to verify
  Action ack_to_verify()
  {
    engine.on_ack(0x06, 0x8a, now);
    auto acts = tick();
    const Action * vg = find(acts, ActionType::SEND_VALGET_VERIFY);
    EXPECT_TRUE(vg != nullptr);
    EXPECT_EQ(engine.state(), State::VERIFY_WAIT);
    return vg ? *vg : Action{};
  }

  std::vector<KeyBytes> reply(const std::vector<KeyId> & keys, bool applied) const
  {
    std::vector<KeyBytes> r;
    for (auto k : keys) {
      r.push_back(KeyBytes{k, applied ? value_of(k) : default_of(k)});
    }
    return r;
  }
};

}  // namespace

TEST(ConfigEngine, nominal_apply_verify_ready) {
  Fixture f;
  Action vs = f.start_to_valset();
  EXPECT_EQ(vs.keys.size(), 4u);
  EXPECT_EQ(vs.txn_action, 0);
  EXPECT_EQ(f.engine.state(), State::APPLY_WAIT_ACK);
  EXPECT_TRUE(f.engine.nominal());

  Action vg = f.ack_to_verify();
  EXPECT_EQ(vg.keys.size(), 4u);

  EXPECT_TRUE(f.engine.on_valget_response(f.reply(vg.keys, true), f.now));
  auto acts = f.tick();
  EXPECT_EQ(Fixture::count(acts, ActionType::MARK_KEYS_VERIFIED), 1);
  EXPECT_TRUE(
    Fixture::has_log(acts, LogLevel::INFO, "user configuration verified on device (4 keys)"));
  EXPECT_EQ(f.engine.state(), State::READY);
}

TEST(ConfigEngine, foreign_valget_reply_is_not_consumed) {
  Fixture f;
  f.start_to_valset();
  Action vg = f.ack_to_verify();
  // a sweep-style reply with different keys must fall through to the generic path
  std::vector<KeyBytes> other = {KeyBytes{0x20910352, {0x00}}};
  EXPECT_FALSE(f.engine.on_valget_response(other, f.now));
  EXPECT_EQ(f.engine.state(), State::VERIFY_WAIT);
  EXPECT_TRUE(f.engine.on_valget_response(f.reply(vg.keys, true), f.now));
}

TEST(ConfigEngine, acked_but_not_applied_walks_the_ladder) {
  Params p;
  Fixture f(p);
  Action vs = f.start_to_valset();

  // rung a: 4 attempts, ACKed but readback shows defaults, backoff 0.5/1/2/4
  double backoffs[] = {0.5, 1.0, 2.0};
  for (int attempt = 1; attempt <= 4; attempt++) {
    EXPECT_EQ(f.engine.rung(), Rung::A);
    EXPECT_EQ(f.engine.attempt(), attempt);
    Action vg = f.ack_to_verify();
    EXPECT_TRUE(f.engine.on_valget_response(f.reply(vg.keys, false), f.now));
    auto acts = f.tick();
    EXPECT_TRUE(Fixture::has_log(acts, LogLevel::WARN, "config NOT applied on device (rung a"));
    EXPECT_TRUE(
      Fixture::has_log(acts, LogLevel::WARN, "CFG_USBOUTPROT_NMEA expected 0x00 got 0x01"));
    if (attempt < 4) {
      EXPECT_EQ(f.engine.state(), State::APPLY_BACKOFF);
      // nothing goes out before the backoff expires
      f.advance(backoffs[attempt - 1] - 0.1);
      EXPECT_EQ(Fixture::count(f.tick(), ActionType::SEND_VALSET), 0);
      f.advance(0.2);
      acts = f.tick();
      const Action * next = Fixture::find(acts, ActionType::SEND_VALSET);
      ASSERT_TRUE(next != nullptr);
      EXPECT_EQ(next->txn_action, 0);
      f.engine.on_valset_sent(f.expected_for(next->keys), f.now);
    } else {
      // rung a exhausted -> rung b starts with a transaction-start frame
      EXPECT_TRUE(Fixture::has_log(acts, LogLevel::WARN, "rung a exhausted"));
      EXPECT_EQ(f.engine.rung(), Rung::B);
      // the transaction-start frame goes out in the same tick that drains the WARN logs
      const Action * txn = Fixture::find(acts, ActionType::SEND_VALSET);
      ASSERT_TRUE(txn != nullptr);
      EXPECT_EQ(txn->txn_action, 1);
      f.engine.on_valset_sent(f.expected_for(txn->keys), f.now);
    }
  }
  EXPECT_FALSE(f.engine.nominal());

  // rung b: start ACK -> apply frame (txn 3) -> ACK -> verify; 2 attempts
  for (int attempt = 1; attempt <= 2; attempt++) {
    EXPECT_EQ(f.engine.state(), State::TXN_START_WAIT_ACK);
    f.engine.on_ack(0x06, 0x8a, f.now);
    auto acts = f.tick();
    const Action * end = Fixture::find(acts, ActionType::SEND_VALSET);
    ASSERT_TRUE(end != nullptr);
    EXPECT_EQ(end->txn_action, 3);
    EXPECT_EQ(end->keys.size(), 4u);
    EXPECT_EQ(f.engine.state(), State::TXN_END_WAIT_ACK);
    Action vg = f.ack_to_verify();
    EXPECT_TRUE(f.engine.on_valget_response(f.reply(vg.keys, false), f.now));
    acts = f.tick();
    EXPECT_TRUE(Fixture::has_log(acts, LogLevel::WARN, "config NOT applied on device (rung b"));
    if (attempt == 1) {
      EXPECT_EQ(f.engine.state(), State::APPLY_BACKOFF);
      f.advance(1.1);
      acts = f.tick();
      const Action * txn = Fixture::find(acts, ActionType::SEND_VALSET);
      ASSERT_TRUE(txn != nullptr);
      EXPECT_EQ(txn->txn_action, 1);
      f.engine.on_valset_sent(f.expected_for(txn->keys), f.now);
    } else {
      // rung c: CFG-RST goes out immediately
      EXPECT_TRUE(Fixture::has_log(acts, LogLevel::WARN, "sending UBX-CFG-RST"));
      EXPECT_EQ(Fixture::count(acts, ActionType::SEND_CFG_RST), 1);
      EXPECT_EQ(f.engine.state(), State::RESET_SETTLE);
      EXPECT_EQ(f.engine.reset_count(), 1);
      EXPECT_TRUE(f.engine.reset_in_progress());
    }
  }

  // the reset re-enumerates USB: detach keeps the counters, re-attach restarts at rung a
  f.engine.on_detached(f.now);
  EXPECT_EQ(f.engine.state(), State::IDLE);
  f.advance(1.0);
  f.engine.start(f.info(), f.now);
  auto acts = f.tick();
  EXPECT_TRUE(Fixture::has_log(acts, LogLevel::WARN, "reopened after UBX-CFG-RST #1"));
  EXPECT_EQ(f.engine.reset_count(), 1);
  EXPECT_EQ(f.engine.rung(), Rung::A);
  f.engine.on_mon_ver(true, kMonVer, f.now);
  acts = f.tick();
  const Action * vs2 = Fixture::find(acts, ActionType::SEND_VALSET);
  ASSERT_TRUE(vs2 != nullptr);
  f.engine.on_valset_sent(f.expected_for(vs2->keys), f.now);

  // this time the receiver applies it
  Action vg = f.ack_to_verify();
  EXPECT_TRUE(f.engine.on_valget_response(f.reply(vg.keys, true), f.now));
  acts = f.tick();
  EXPECT_TRUE(
    Fixture::has_log(acts, LogLevel::INFO, "verified on device (4 keys) (after 1 reset(s)"));
  EXPECT_EQ(f.engine.state(), State::READY);
  EXPECT_TRUE(f.engine.nominal());
  EXPECT_EQ(f.engine.reset_count(), 0);
  (void)vs;
}

TEST(ConfigEngine, reset_without_reenumeration_rehandshakes_and_degrades) {
  Params p;
  p.valset_attempts = 1;
  p.txn_attempts = 0;
  p.reset_attempts = 2;
  p.reset_settle_s = 3.0;
  p.degraded_retry_s = 30.0;
  Fixture f(p);

  auto fail_once = [&]() {
      Action vg = f.ack_to_verify();
      EXPECT_TRUE(f.engine.on_valget_response(f.reply(vg.keys, false), f.now));
      return f.tick();
    };

  f.start_to_valset();
  auto acts = fail_once();
  EXPECT_EQ(Fixture::count(acts, ActionType::SEND_CFG_RST), 1);
  EXPECT_EQ(f.engine.state(), State::RESET_SETTLE);

  // no detach: after the settle time the engine re-handshakes on the same connection
  f.advance(2.9);
  EXPECT_EQ(Fixture::count(f.tick(), ActionType::POLL_MON_VER), 0);
  f.advance(0.2);
  acts = f.tick();
  EXPECT_TRUE(Fixture::has_log(acts, LogLevel::WARN, "no serial port loss"));
  EXPECT_EQ(Fixture::count(acts, ActionType::POLL_MON_VER), 1);
  EXPECT_EQ(f.engine.state(), State::HANDSHAKE);
  f.engine.on_mon_ver(true, kMonVer, f.now);
  acts = f.tick();
  const Action * vs = Fixture::find(acts, ActionType::SEND_VALSET);
  ASSERT_TRUE(vs != nullptr);
  f.engine.on_valset_sent(f.expected_for(vs->keys), f.now);

  // second reset, then rung d
  acts = fail_once();
  EXPECT_EQ(Fixture::count(acts, ActionType::SEND_CFG_RST), 1);
  EXPECT_EQ(f.engine.reset_count(), 2);
  f.advance(3.1);
  acts = f.tick();
  f.engine.on_mon_ver(true, kMonVer, f.now);
  acts = f.tick();
  vs = Fixture::find(acts, ActionType::SEND_VALSET);
  ASSERT_TRUE(vs != nullptr);
  f.engine.on_valset_sent(f.expected_for(vs->keys), f.now);
  acts = fail_once();
  EXPECT_TRUE(Fixture::has_log(acts, LogLevel::ERROR, "refuses configuration after 2 reset(s)"));
  EXPECT_EQ(f.engine.state(), State::DEGRADED);
  EXPECT_EQ(f.engine.rung(), Rung::D);

  // rung d: nothing for 30 s, then one attempt at ERROR level
  f.advance(29.0);
  EXPECT_EQ(Fixture::count(f.tick(), ActionType::SEND_VALSET), 0);
  f.advance(1.5);
  acts = f.tick();
  EXPECT_TRUE(Fixture::has_log(acts, LogLevel::ERROR, "still refuses configuration"));
  vs = Fixture::find(acts, ActionType::SEND_VALSET);
  ASSERT_TRUE(vs != nullptr);
  f.engine.on_valset_sent(f.expected_for(vs->keys), f.now);
  // ... and this time it works: recovered
  Action vg = f.ack_to_verify();
  EXPECT_TRUE(f.engine.on_valget_response(f.reply(vg.keys, true), f.now));
  acts = f.tick();
  EXPECT_TRUE(Fixture::has_log(acts, LogLevel::INFO, "recovered after"));
  EXPECT_EQ(f.engine.state(), State::READY);
  EXPECT_TRUE(f.engine.nominal());
}

TEST(ConfigEngine, nak_and_ack_timeout_retry_then_isolate_bad_key) {
  Fixture f;
  f.start_to_valset();

  // attempt 1: no ACK within 1.5 s -> backoff
  f.advance(1.6);
  auto acts = f.tick();
  EXPECT_TRUE(Fixture::has_log(acts, LogLevel::WARN, "no ACK for CFG-VALSET"));
  EXPECT_EQ(f.engine.state(), State::APPLY_BACKOFF);
  f.advance(0.6);
  acts = f.tick();
  const Action * vs = Fixture::find(acts, ActionType::SEND_VALSET);
  ASSERT_TRUE(vs != nullptr);
  EXPECT_EQ(f.engine.attempt(), 2);
  f.engine.on_valset_sent(f.expected_for(vs->keys), f.now);

  // attempt 2: NAK on the multi-key frame -> isolate keys one per frame
  f.engine.on_nak(0x06, 0x8a, f.now);
  acts = f.tick();
  EXPECT_TRUE(Fixture::has_log(acts, LogLevel::WARN, "isolating keys"));
  vs = Fixture::find(acts, ActionType::SEND_VALSET);
  ASSERT_TRUE(vs != nullptr);
  EXPECT_EQ(vs->keys.size(), 1u);
  EXPECT_EQ(vs->keys[0], K_NMEA);

  // keys 1..3 ACK, the offset key NAKs on its own -> dropped, others verified
  for (int i = 0; i < 3; i++) {
    f.engine.on_valset_sent(f.expected_for(vs->keys), f.now);
    f.engine.on_ack(0x06, 0x8a, f.now);
    acts = f.tick();
    vs = Fixture::find(acts, ActionType::SEND_VALSET);
    ASSERT_TRUE(vs != nullptr);
    EXPECT_EQ(vs->keys.size(), 1u);
  }
  EXPECT_EQ(vs->keys[0], K_OFFSET);
  f.engine.on_valset_sent(f.expected_for(vs->keys), f.now);
  f.engine.on_nak(0x06, 0x8a, f.now);
  acts = f.tick();
  EXPECT_TRUE(Fixture::has_log(acts, LogLevel::ERROR, "NAKed CFG_NAVSPG_DAHEADING_OFFSET"));
  const Action * nak = Fixture::find(acts, ActionType::MARK_KEYS_ACKNAK);
  ASSERT_TRUE(nak != nullptr);
  EXPECT_EQ(nak->keys, std::vector<KeyId>{K_OFFSET});
  const Action * vg = Fixture::find(acts, ActionType::SEND_VALGET_VERIFY);
  ASSERT_TRUE(vg != nullptr);
  EXPECT_EQ(vg->keys.size(), 3u);
  EXPECT_TRUE(f.engine.on_valget_response(f.reply(vg->keys, true), f.now));
  acts = f.tick();
  EXPECT_TRUE(Fixture::has_log(acts, LogLevel::INFO, "verified on device (3 keys)"));
  EXPECT_EQ(f.engine.state(), State::READY);
  EXPECT_EQ(f.engine.user_key_count(), 3u);
  EXPECT_EQ(Fixture::count(acts, ActionType::SEND_CFG_RST), 0);
}

TEST(ConfigEngine, watchdog_trips_on_nav_silence_and_nmea_leak) {
  Fixture f;
  f.start_to_valset();
  Action vg = f.ack_to_verify();
  f.engine.on_valget_response(f.reply(vg.keys, true), f.now);
  f.tick();
  ASSERT_EQ(f.engine.state(), State::READY);

  // NAV frames keep coming: no trip
  for (int i = 0; i < 100; i++) {
    f.advance(0.1);
    if (i % 10 == 0) {f.engine.on_nav_frame(f.now);}
    f.tick();
  }
  EXPECT_EQ(f.engine.state(), State::READY);

  // silence for > 5 s: trip -> VERIFY; readback still good -> READY again, no ladder
  f.advance(5.1);
  auto acts = f.tick();
  EXPECT_TRUE(Fixture::has_log(acts, LogLevel::WARN, "no UBX-NAV message"));
  const Action * v = Fixture::find(acts, ActionType::SEND_VALGET_VERIFY);
  ASSERT_TRUE(v != nullptr);
  EXPECT_FALSE(f.engine.nominal());
  EXPECT_TRUE(f.engine.on_valget_response(f.reply(v->keys, true), f.now));
  acts = f.tick();
  EXPECT_EQ(f.engine.state(), State::READY);
  EXPECT_TRUE(f.engine.nominal());

  // within the 30 s min interval nothing trips even without NAV
  f.advance(6.0);
  EXPECT_EQ(Fixture::count(f.tick(), ActionType::SEND_VALGET_VERIFY), 0);

  // later: NAV fine but NMEA floods at 40/s with NMEA disabled -> trip after a summary window
  f.advance(30.0);
  f.engine.on_nav_frame(f.now);
  bool tripped = false;
  for (int i = 0; i < 140 && !tripped; i++) {
    f.advance(0.05);
    f.engine.on_nav_frame(f.now);
    f.engine.on_nmea(2, "$GNGGA,...", f.now);
    acts = f.tick();
    if (Fixture::has_log(acts, LogLevel::WARN, "NMEA still streaming")) {
      tripped = true;
      EXPECT_TRUE(Fixture::find(acts, ActionType::SEND_VALGET_VERIFY) != nullptr);
    }
  }
  EXPECT_TRUE(tripped);
}

TEST(ConfigEngine, nmea_leak_at_one_per_second_is_tolerated_and_summarised) {
  Fixture f;
  f.start_to_valset();
  Action vg = f.ack_to_verify();
  f.engine.on_valget_response(f.reply(vg.keys, true), f.now);
  f.tick();
  bool summary = false;
  for (int i = 0; i < 260; i++) {
    f.advance(0.05);
    if (i % 20 == 0) {
      f.engine.on_nav_frame(f.now);
      f.engine.on_nmea(1, "$GNTHS,64.25,A", f.now);
    }
    auto acts = f.tick();
    if (Fixture::has_log(acts, LogLevel::INFO, "nmea: ")) {
      summary = true;
      EXPECT_TRUE(Fixture::has_log(acts, LogLevel::INFO, "last: $GNTHS,64.25,A"));
    }
    EXPECT_FALSE(Fixture::has_log(acts, LogLevel::WARN, "NMEA still streaming"));
  }
  EXPECT_TRUE(summary);
  EXPECT_EQ(f.engine.state(), State::READY);
}

TEST(ConfigEngine, port_loss_mid_ladder_resets_state) {
  Fixture f;
  Action vs = f.start_to_valset();
  EXPECT_EQ(vs.keys.size(), 4u);

  // port lost with a VALSET in flight: engine idles, late ACKs are ignored
  f.engine.on_detached(f.now);
  EXPECT_EQ(f.engine.state(), State::IDLE);
  EXPECT_FALSE(f.engine.started());
  f.engine.on_ack(0x06, 0x8a, f.now);
  EXPECT_EQ(f.engine.state(), State::IDLE);
  EXPECT_EQ(Fixture::count(f.tick(), ActionType::SEND_VALSET), 0);

  // reopen without a pending reset: fresh ladder
  f.engine.start(f.info(), f.now);
  EXPECT_EQ(f.engine.state(), State::HANDSHAKE);
  EXPECT_FALSE(f.engine.reset_in_progress());
  f.tick();
  f.engine.on_mon_ver(true, kMonVer, f.now);
  auto acts = f.tick();
  const Action * vs2 = Fixture::find(acts, ActionType::SEND_VALSET);
  ASSERT_TRUE(vs2 != nullptr);
  f.engine.on_valset_sent(f.expected_for(vs2->keys), f.now);
  Action vg = f.ack_to_verify();
  EXPECT_TRUE(f.engine.on_valget_response(f.reply(vg.keys, true), f.now));
  f.tick();
  EXPECT_EQ(f.engine.state(), State::READY);
  EXPECT_TRUE(f.engine.nominal());
}

TEST(ConfigEngine, incompatible_receiver_is_never_configured) {
  Fixture f;
  f.engine.start(f.info(), f.now);
  f.tick();
  f.engine.on_mon_ver(false, "MOD=ZED-F9P FWVER=HPG 1.32", f.now);
  auto acts = f.tick();
  EXPECT_TRUE(Fixture::has_log(acts, LogLevel::ERROR, "configuration disabled"));
  EXPECT_EQ(f.engine.state(), State::INCOMPATIBLE);
  for (int i = 0; i < 100; i++) {
    f.advance(1.0);
    acts = f.tick();
    EXPECT_EQ(Fixture::count(acts, ActionType::SEND_VALSET), 0);
    EXPECT_EQ(Fixture::count(acts, ActionType::POLL_MON_VER), 0);
  }
}

TEST(ConfigEngine, no_reset_when_receiver_reset_not_allowed) {
  // allow_receiver_reset = false maps to reset_attempts = 0: after rung b the engine
  // goes to rung d without ever sending UBX-CFG-RST.
  Params p;
  p.valset_attempts = 1;
  p.txn_attempts = 1;
  p.reset_attempts = 0;
  Fixture f(p);
  f.start_to_valset();

  Action vg = f.ack_to_verify();
  EXPECT_TRUE(f.engine.on_valget_response(f.reply(vg.keys, false), f.now));
  auto acts = f.tick();
  EXPECT_EQ(f.engine.rung(), Rung::B);
  const Action * txn = Fixture::find(acts, ActionType::SEND_VALSET);
  ASSERT_TRUE(txn != nullptr);
  EXPECT_EQ(txn->txn_action, 1);
  f.engine.on_valset_sent(f.expected_for(txn->keys), f.now);
  f.engine.on_ack(0x06, 0x8a, f.now);
  acts = f.tick();
  const Action * end = Fixture::find(acts, ActionType::SEND_VALSET);
  ASSERT_TRUE(end != nullptr);
  EXPECT_EQ(end->txn_action, 3);
  vg = f.ack_to_verify();
  EXPECT_TRUE(f.engine.on_valget_response(f.reply(vg.keys, false), f.now));
  acts = f.tick();
  EXPECT_EQ(Fixture::count(acts, ActionType::SEND_CFG_RST), 0);
  EXPECT_EQ(f.engine.state(), State::DEGRADED);
  EXPECT_EQ(f.engine.rung(), Rung::D);
  for (int i = 0; i < 200; i++) {
    f.advance(1.0);
    EXPECT_EQ(Fixture::count(f.tick(), ActionType::SEND_CFG_RST), 0);
  }
}

TEST(ConfigEngine, handshake_timeout_proceeds_with_warning) {
  Fixture f;
  f.engine.start(f.info(), f.now);
  auto acts = f.tick();
  EXPECT_EQ(Fixture::count(acts, ActionType::POLL_MON_VER), 1);
  int polls = 0;
  bool warned = false;
  for (int i = 0; i < 120; i++) {
    f.advance(0.05);
    acts = f.tick();
    polls += Fixture::count(acts, ActionType::POLL_MON_VER);
    if (Fixture::has_log(acts, LogLevel::WARN, "no MON-VER reply")) {
      warned = true;
      EXPECT_TRUE(Fixture::find(acts, ActionType::SEND_VALSET) != nullptr);
      break;
    }
  }
  EXPECT_TRUE(warned);
  EXPECT_GE(polls, 4);
  EXPECT_EQ(f.engine.state(), State::APPLY_WAIT_ACK);
}

TEST(ConfigEngine, more_than_64_keys_are_chunked_and_transaction_uses_ongoing_frames) {
  Params p;
  p.valset_attempts = 1;
  Fixture f(p);
  StartInfo info;
  for (KeyId k = 0x20910000; k < 0x20910000 + 70; k++) {
    info.user_keys.push_back({k, "K"});
  }
  f.engine.start(info, f.now);
  f.tick();
  f.engine.on_mon_ver(true, kMonVer, f.now);
  auto acts = f.tick();
  const Action * vs = Fixture::find(acts, ActionType::SEND_VALSET);
  ASSERT_TRUE(vs != nullptr);
  EXPECT_EQ(vs->keys.size(), 64u);
  f.engine.on_ack(0x06, 0x8a, f.now);
  acts = f.tick();
  vs = Fixture::find(acts, ActionType::SEND_VALSET);
  ASSERT_TRUE(vs != nullptr);
  EXPECT_EQ(vs->keys.size(), 6u);
  f.engine.on_ack(0x06, 0x8a, f.now);
  acts = f.tick();
  const Action * vg = Fixture::find(acts, ActionType::SEND_VALGET_VERIFY);
  ASSERT_TRUE(vg != nullptr);
  EXPECT_EQ(vg->keys.size(), 64u);
  // fail verification straight into rung b: start(64) -> ongoing(6) -> apply(6)
  std::vector<KeyBytes> bad;
  for (auto k : vg->keys) {
    bad.push_back(KeyBytes{k, {0x07}});
  }
  std::vector<KeyBytes> exp;
  for (auto k : vg->keys) {
    exp.push_back(KeyBytes{k, {0x01}});
  }
  f.engine.on_valset_sent(exp, f.now);
  EXPECT_TRUE(f.engine.on_valget_response(bad, f.now));
  acts = f.tick();
  EXPECT_EQ(f.engine.rung(), Rung::B);
  vs = Fixture::find(acts, ActionType::SEND_VALSET);
  ASSERT_TRUE(vs != nullptr);
  EXPECT_EQ(vs->txn_action, 1);
  EXPECT_EQ(vs->keys.size(), 64u);
  f.engine.on_ack(0x06, 0x8a, f.now);
  acts = f.tick();
  vs = Fixture::find(acts, ActionType::SEND_VALSET);
  ASSERT_TRUE(vs != nullptr);
  EXPECT_EQ(vs->txn_action, 2);
  EXPECT_EQ(vs->keys.size(), 6u);
  f.engine.on_ack(0x06, 0x8a, f.now);
  acts = f.tick();
  vs = Fixture::find(acts, ActionType::SEND_VALSET);
  ASSERT_TRUE(vs != nullptr);
  EXPECT_EQ(vs->txn_action, 3);
  EXPECT_EQ(vs->keys.size(), 6u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
