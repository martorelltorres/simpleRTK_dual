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

// Payloads are built by hand at the documented byte offsets, independently of the
// parser, so that an offset error in either shows up as a field mismatch.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "ublox_x20d_driver/ubx_types.h"

using namespace ublox_x20d_driver::ubx;
using Bytes = std::vector<uint8_t>;

namespace
{
void put_u1(Bytes& b, size_t offset, uint8_t v)
{
  b.at(offset) = v;
}

void put_u2(Bytes& b, size_t offset, uint16_t v)
{
  b.at(offset) = static_cast<uint8_t>(v);
  b.at(offset + 1) = static_cast<uint8_t>(v >> 8);
}

void put_u4(Bytes& b, size_t offset, uint32_t v)
{
  for (size_t i = 0; i < 4; ++i)
  {
    b.at(offset + i) = static_cast<uint8_t>(v >> (8 * i));
  }
}

void put_i4(Bytes& b, size_t offset, int32_t v)
{
  put_u4(b, offset, static_cast<uint32_t>(v));
}

Bytes daheading_payload(uint32_t flags)
{
  Bytes b(kNavDAHeadingLength, 0xEE);  // reserved bytes keep a non-zero filler
  put_u1(b, 0, 0x02);
  put_u4(b, 4, 345600000);
  put_i4(b, 8, -812);
  put_i4(b, 12, 583);
  put_i4(b, 16, -17);
  put_i4(b, 20, 1000);
  put_i4(b, 24, 14432100);  // 144.321 deg
  put_u4(b, 32, 11);
  put_u4(b, 36, 12);
  put_u4(b, 40, 13);
  put_u4(b, 44, 14);
  put_u4(b, 48, 61000);  // 0.61 deg
  put_u4(b, 56, flags);
  return b;
}
}  // namespace

// --- little-endian primitives ---------------------------------------------------

TEST(LittleEndian, SignedValues)
{
  const Bytes b{ 0xFF, 0xFF, 0xFF, 0xFF };
  EXPECT_EQ(read_i4(b.data()), -1);
  EXPECT_EQ(read_i2(b.data()), -1);
  EXPECT_EQ(read_i1(b.data()), -1);

  Bytes out;
  append_u4(out, static_cast<uint32_t>(-9000));
  EXPECT_EQ(read_i4(out.data()), -9000);
}

TEST(LittleEndian, ByteOrder)
{
  const Bytes b{ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
  EXPECT_EQ(read_u2(b.data()), 0x0201u);
  EXPECT_EQ(read_u4(b.data()), 0x04030201u);
  EXPECT_EQ(read_u8(b.data()), 0x0807060504030201ull);

  Bytes out;
  append_u8(out, 0x0807060504030201ull);
  EXPECT_EQ(out, b);
}

// --- NAV-DAHEADING --------------------------------------------------------------

TEST(NavDAHeading, AllFields)
{
  NavDAHeading m{};
  ASSERT_EQ(parse_nav_daheading(daheading_payload(0), &m), ParseResult::kOk);
  EXPECT_EQ(m.version, 0x02);
  EXPECT_EQ(m.itow, 345600000u);
  EXPECT_EQ(m.rel_pos_n, -812);
  EXPECT_EQ(m.rel_pos_e, 583);
  EXPECT_EQ(m.rel_pos_d, -17);
  EXPECT_EQ(m.rel_pos_length, 1000);
  EXPECT_EQ(m.rel_pos_heading, 14432100);
  EXPECT_EQ(m.acc_n, 11u);
  EXPECT_EQ(m.acc_e, 12u);
  EXPECT_EQ(m.acc_d, 13u);
  EXPECT_EQ(m.acc_length, 14u);
  EXPECT_EQ(m.acc_heading, 61000u);
  EXPECT_EQ(m.flags, 0u);
}

TEST(NavDAHeading, FlagsNoneSet)
{
  NavDAHeading m{};
  ASSERT_EQ(parse_nav_daheading(daheading_payload(0), &m), ParseResult::kOk);
  EXPECT_FALSE(m.gnss_fix_ok);
  EXPECT_FALSE(m.diff_soln);
  EXPECT_FALSE(m.rel_pos_valid);
  EXPECT_EQ(m.carr_soln, kCarrSolnNone);
  EXPECT_FALSE(m.rel_pos_heading_valid);
}

TEST(NavDAHeading, FlagsOneBitAtATime)
{
  NavDAHeading m{};

  ASSERT_EQ(parse_nav_daheading(daheading_payload(1u << 0), &m), ParseResult::kOk);
  EXPECT_TRUE(m.gnss_fix_ok);
  EXPECT_FALSE(m.diff_soln || m.rel_pos_valid || m.rel_pos_heading_valid || m.carr_soln);

  ASSERT_EQ(parse_nav_daheading(daheading_payload(1u << 1), &m), ParseResult::kOk);
  EXPECT_TRUE(m.diff_soln);
  EXPECT_FALSE(m.gnss_fix_ok || m.rel_pos_valid || m.rel_pos_heading_valid || m.carr_soln);

  ASSERT_EQ(parse_nav_daheading(daheading_payload(1u << 2), &m), ParseResult::kOk);
  EXPECT_TRUE(m.rel_pos_valid);
  EXPECT_FALSE(m.gnss_fix_ok || m.diff_soln || m.rel_pos_heading_valid || m.carr_soln);

  ASSERT_EQ(parse_nav_daheading(daheading_payload(1u << 6), &m), ParseResult::kOk);
  EXPECT_TRUE(m.rel_pos_heading_valid);
  EXPECT_FALSE(m.gnss_fix_ok || m.diff_soln || m.rel_pos_valid || m.carr_soln);

  // Bit 5 is reserved and bit 8 is relPosHeadingValid in NAV-RELPOSNED, not here.
  ASSERT_EQ(parse_nav_daheading(daheading_payload((1u << 5) | (1u << 8)), &m), ParseResult::kOk);
  EXPECT_FALSE(m.gnss_fix_ok || m.diff_soln || m.rel_pos_valid || m.rel_pos_heading_valid || m.carr_soln);
  EXPECT_EQ(m.flags, (1u << 5) | (1u << 8));
}

TEST(NavDAHeading, CarrierSolution)
{
  NavDAHeading m{};
  ASSERT_EQ(parse_nav_daheading(daheading_payload(1u << 3), &m), ParseResult::kOk);
  EXPECT_EQ(m.carr_soln, kCarrSolnFloat);
  ASSERT_EQ(parse_nav_daheading(daheading_payload(2u << 3), &m), ParseResult::kOk);
  EXPECT_EQ(m.carr_soln, kCarrSolnFixed);
  // A typical good epoch: fix ok, relPos valid, fixed, heading valid.
  ASSERT_EQ(parse_nav_daheading(daheading_payload(0x01 | 0x04 | (2u << 3) | 0x40), &m), ParseResult::kOk);
  EXPECT_TRUE(m.gnss_fix_ok && m.rel_pos_valid && m.rel_pos_heading_valid);
  EXPECT_FALSE(m.diff_soln);
  EXPECT_EQ(m.carr_soln, kCarrSolnFixed);
}

TEST(NavDAHeading, RejectsPreProductionVersion)
{
  Bytes b(64, 0);
  b[0] = 0x01;
  NavDAHeading m{};
  EXPECT_EQ(parse_nav_daheading(b, &m), ParseResult::kUnsupportedVersion);
}

TEST(NavDAHeading, RejectsShortPayload)
{
  Bytes b = daheading_payload(0);
  b.pop_back();
  NavDAHeading m{};
  EXPECT_EQ(parse_nav_daheading(b, &m), ParseResult::kTooShort);
  EXPECT_EQ(parse_nav_daheading(Bytes{}, &m), ParseResult::kTooShort);
}

// --- NAV-PVT --------------------------------------------------------------------

TEST(NavPvt, AllFields)
{
  Bytes b(kNavPvtLength, 0);
  put_u4(b, 0, 345600000);
  put_u2(b, 4, 2026);
  put_u1(b, 6, 9);
  put_u1(b, 7, 18);
  put_u1(b, 8, 13);
  put_u1(b, 9, 45);
  put_u1(b, 10, 59);
  put_u1(b, 11, 0x07);
  put_u4(b, 12, 25);
  put_i4(b, 16, -1234);
  put_u1(b, 20, 3);
  put_u1(b, 21, 0x01 | 0x02 | (2u << 6));
  put_u1(b, 23, 31);
  put_i4(b, 24, 26461234);   // lon
  put_i4(b, 28, 395712345);  // lat
  put_i4(b, 32, 51234);
  put_i4(b, 36, 1234);
  put_u4(b, 40, 14);
  put_u4(b, 44, 21);
  put_i4(b, 48, 100);
  put_i4(b, 52, -200);
  put_i4(b, 56, 30);
  put_i4(b, 60, 223);
  put_i4(b, 64, 29000000);
  put_u4(b, 68, 40);
  put_u4(b, 72, 500000);
  put_u2(b, 76, 123);
  put_u1(b, 78, 0x01);

  NavPvt m{};
  ASSERT_EQ(parse_nav_pvt(b, &m), ParseResult::kOk);
  EXPECT_EQ(m.itow, 345600000u);
  EXPECT_EQ(m.year, 2026);
  EXPECT_EQ(m.month, 9);
  EXPECT_EQ(m.day, 18);
  EXPECT_EQ(m.hour, 13);
  EXPECT_EQ(m.min, 45);
  EXPECT_EQ(m.sec, 59);
  EXPECT_TRUE(m.valid_date && m.valid_time && m.fully_resolved);
  EXPECT_EQ(m.t_acc, 25u);
  EXPECT_EQ(m.nano, -1234);
  EXPECT_EQ(m.fix_type, 3);
  EXPECT_TRUE(m.gnss_fix_ok);
  EXPECT_TRUE(m.diff_soln);
  EXPECT_EQ(m.carr_soln, kCarrSolnFixed);
  EXPECT_EQ(m.num_sv, 31);
  EXPECT_EQ(m.lon, 26461234);
  EXPECT_EQ(m.lat, 395712345);
  EXPECT_EQ(m.height, 51234);
  EXPECT_EQ(m.hmsl, 1234);
  EXPECT_EQ(m.h_acc, 14u);
  EXPECT_EQ(m.v_acc, 21u);
  EXPECT_EQ(m.vel_n, 100);
  EXPECT_EQ(m.vel_e, -200);
  EXPECT_EQ(m.vel_d, 30);
  EXPECT_EQ(m.g_speed, 223);
  EXPECT_EQ(m.head_mot, 29000000);
  EXPECT_EQ(m.s_acc, 40u);
  EXPECT_EQ(m.head_acc, 500000u);
  EXPECT_EQ(m.p_dop, 123);
  EXPECT_TRUE(m.invalid_llh);
}

TEST(NavPvt, CarrierSolutionDoesNotLeakIntoOtherFlags)
{
  Bytes b(kNavPvtLength, 0);
  put_u1(b, 21, 1u << 6);
  NavPvt m{};
  ASSERT_EQ(parse_nav_pvt(b, &m), ParseResult::kOk);
  EXPECT_EQ(m.carr_soln, kCarrSolnFloat);
  EXPECT_FALSE(m.gnss_fix_ok || m.diff_soln);
}

TEST(NavPvt, LengthHandling)
{
  NavPvt m{};
  EXPECT_EQ(parse_nav_pvt(Bytes(kNavPvtLength - 1, 0), &m), ParseResult::kTooShort);
  EXPECT_EQ(parse_nav_pvt(Bytes(kNavPvtLength + 8, 0), &m), ParseResult::kOk);
}

// --- NAV-HPPOSLLH ---------------------------------------------------------------

TEST(NavHPPosLLH, AllFields)
{
  Bytes b(kNavHPPosLLHLength, 0);
  put_u1(b, 0, 0x00);
  put_u1(b, 3, 0x01);
  put_u4(b, 4, 345600000);
  put_i4(b, 8, 26461234);
  put_i4(b, 12, 395712345);
  put_i4(b, 16, 51234);
  put_i4(b, 20, 1234);
  put_u1(b, 24, static_cast<uint8_t>(-45));
  put_u1(b, 25, 67);
  put_u1(b, 26, static_cast<uint8_t>(-9));
  put_u1(b, 27, 8);
  put_u4(b, 28, 141);
  put_u4(b, 32, 212);

  NavHPPosLLH m{};
  ASSERT_EQ(parse_nav_hpposllh(b, &m), ParseResult::kOk);
  EXPECT_EQ(m.version, 0);
  EXPECT_TRUE(m.invalid_llh);
  EXPECT_EQ(m.itow, 345600000u);
  EXPECT_EQ(m.lon, 26461234);
  EXPECT_EQ(m.lat, 395712345);
  EXPECT_EQ(m.height, 51234);
  EXPECT_EQ(m.hmsl, 1234);
  EXPECT_EQ(m.lon_hp, -45);
  EXPECT_EQ(m.lat_hp, 67);
  EXPECT_EQ(m.height_hp, -9);
  EXPECT_EQ(m.hmsl_hp, 8);
  EXPECT_EQ(m.h_acc, 141u);
  EXPECT_EQ(m.v_acc, 212u);
}

TEST(NavHPPosLLH, RejectsShortPayload)
{
  NavHPPosLLH m{};
  EXPECT_EQ(parse_nav_hpposllh(Bytes(kNavHPPosLLHLength - 1, 0), &m), ParseResult::kTooShort);
}

// --- iTOW ---------------------------------------------------------------------

TEST(NavItow, OffsetsPerMessage)
{
  uint32_t itow = 0;
  Bytes pvt(kNavPvtLength, 0);
  put_u4(pvt, 0, 1000);
  EXPECT_TRUE(nav_itow(Frame{ msg_class::kNav, msg_id::kNavPvt, pvt }, &itow));
  EXPECT_EQ(itow, 1000u);
  Bytes hp(kNavHPPosLLHLength, 0);
  put_u4(hp, 4, 2000);
  EXPECT_TRUE(nav_itow(Frame{ msg_class::kNav, msg_id::kNavHPPosLLH, hp }, &itow));
  EXPECT_EQ(itow, 2000u);
  EXPECT_TRUE(nav_itow(Frame{ msg_class::kNav, msg_id::kNavDAHeading, daheading_payload(0) }, &itow));
  EXPECT_EQ(itow, 345600000u);
  EXPECT_FALSE(nav_itow(Frame{ msg_class::kRxm, msg_id::kRxmRawx, Bytes(64, 0) }, &itow));
  EXPECT_FALSE(nav_itow(Frame{ msg_class::kNav, msg_id::kNavDAHeading, Bytes(7, 0) }, &itow));
}

// --- MON-VER --------------------------------------------------------------------

namespace
{
Bytes mon_ver_payload(const std::vector<std::string>& extensions)
{
  Bytes b(40, 0);
  const std::string sw = "EXT CORE 2.00 (abcdef)";
  const std::string hw = "000C0000";
  std::copy(sw.begin(), sw.end(), b.begin());
  std::copy(hw.begin(), hw.end(), b.begin() + 30);
  for (const std::string& ext : extensions)
  {
    Bytes field(30, 0);
    std::copy(ext.begin(), ext.end(), field.begin());
    b.insert(b.end(), field.begin(), field.end());
  }
  return b;
}
}  // namespace

TEST(MonVer, ParsesStrings)
{
  MonVer v;
  ASSERT_EQ(parse_mon_ver(mon_ver_payload({ "FWVER=HDG 2.00", "PROTVER=57.02", "MOD=ZED-X20D" }), &v),
            ParseResult::kOk);
  EXPECT_EQ(v.sw_version, "EXT CORE 2.00 (abcdef)");
  EXPECT_EQ(v.hw_version, "000C0000");
  ASSERT_EQ(v.extensions.size(), 3u);
  EXPECT_EQ(mon_ver_field(v, "PROTVER"), "57.02");
  EXPECT_EQ(mon_ver_field(v, "MISSING"), "");
  EXPECT_EQ(mon_ver_summary(v), "MOD=ZED-X20D FWVER=HDG 2.00 PROTVER=57.02");
}

TEST(MonVer, Compatibility)
{
  MonVer v;
  ASSERT_EQ(parse_mon_ver(mon_ver_payload({ "FWVER=HDG 2.00", "MOD=ZED-X20D" }), &v), ParseResult::kOk);
  EXPECT_TRUE(mon_ver_compatible(v));
  ASSERT_EQ(parse_mon_ver(mon_ver_payload({ "FWVER=HDG 2.10", "MOD=ZED-X20D" }), &v), ParseResult::kOk);
  EXPECT_TRUE(mon_ver_compatible(v));
  ASSERT_EQ(parse_mon_ver(mon_ver_payload({ "FWVER=HDG 1.90", "MOD=ZED-X20D" }), &v), ParseResult::kOk);
  EXPECT_FALSE(mon_ver_compatible(v));
  ASSERT_EQ(parse_mon_ver(mon_ver_payload({ "FWVER=HPG 2.00", "MOD=ZED-X20D" }), &v), ParseResult::kOk);
  EXPECT_FALSE(mon_ver_compatible(v));
  ASSERT_EQ(parse_mon_ver(mon_ver_payload({ "FWVER=HPG 1.32", "MOD=ZED-F9P" }), &v), ParseResult::kOk);
  EXPECT_FALSE(mon_ver_compatible(v));
  ASSERT_EQ(parse_mon_ver(mon_ver_payload({ "MOD=ZED-X20D" }), &v), ParseResult::kOk);
  EXPECT_FALSE(mon_ver_compatible(v));
}

TEST(MonVer, RejectsShortPayload)
{
  MonVer v;
  EXPECT_EQ(parse_mon_ver(Bytes(39, 0), &v), ParseResult::kTooShort);
}

namespace
{
Bytes from_hex(const std::string& hex)
{
  Bytes b;
  for (size_t i = 0; i + 1 < hex.size(); i += 2)
  {
    b.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
  }
  return b;
}

// RXM-RAWX recorded from a ZED-X20D (HDG 2.00): the real header and its first two
// measurements, with numMeas set to 2. Expected values decoded independently with pyubx2.
const Bytes kRawxPayload = from_hex(
    "dd2406015cd516418609120241021d6d"
    "4634c81cea477441abc349f68be59341"
    "2f7ab14400060700f4fb2c0402070700"
    "32aef0886a5273415190b221b3f49241"
    "1d5817440009070024361c0507080700");

// RXM-SFRBX recorded from the same receiver (GPS SV 9, 10 words).
const Bytes kSfrbxPayload = from_hex(
    "000900000a9b02200986c122d3896d9e3d0064182ed9189d18a334abaf6c550f"
    "4600f33818dad693b3ea3f004721c815");
}  // namespace

TEST(RxmRawx, RecordedFrame)
{
  RxmRawx r;
  ASSERT_EQ(parse_rxm_rawx(kRawxPayload, &r), ParseResult::kOk);
  EXPECT_DOUBLE_EQ(r.rcv_tow, 374103.001);
  EXPECT_EQ(r.week, 2438);
  EXPECT_EQ(r.leap_s, 18);
  EXPECT_EQ(r.rec_stat, 0x41);  // bit 0 leapSec; bit 6 is set on HDG 2.00 but not documented
  EXPECT_EQ(r.version, 2);
  EXPECT_EQ(r.reserved0, 27933);
  ASSERT_EQ(r.meas.size(), 2u);

  const RawxMeas& a = r.meas[0];
  EXPECT_DOUBLE_EQ(a.pr_mes, 21266081.798877977);
  EXPECT_DOUBLE_EQ(a.cp_mes, 83452669.57203548);
  EXPECT_FLOAT_EQ(a.do_mes, 1419.8182373046875f);
  EXPECT_EQ(a.gnss_id, 0);
  EXPECT_EQ(a.sv_id, 6);
  EXPECT_EQ(a.sig_id, 7);
  EXPECT_EQ(a.freq_id, 0);
  EXPECT_EQ(a.locktime, 64500);
  EXPECT_EQ(a.cno, 44);
  EXPECT_EQ(a.pr_stdev, 4);
  EXPECT_EQ(a.cp_stdev, 2);
  EXPECT_EQ(a.do_stdev, 7);
  EXPECT_EQ(a.trk_stat, 0x07);  // prValid, cpValid, halfCyc

  const RawxMeas& b = r.meas[1];
  EXPECT_DOUBLE_EQ(b.pr_mes, 20260520.558759876);
  EXPECT_DOUBLE_EQ(b.cp_mes, 79506632.42437865);
  EXPECT_FLOAT_EQ(b.do_mes, 605.3767700195312f);
  EXPECT_EQ(b.sv_id, 9);
  EXPECT_EQ(b.locktime, 13860);
  EXPECT_EQ(b.cno, 28);
  EXPECT_EQ(b.pr_stdev, 5);
  EXPECT_EQ(b.cp_stdev, 7);
  EXPECT_EQ(b.do_stdev, 8);
}

TEST(RxmRawx, EncodeRebuildsTheRecordedPayload)
{
  RxmRawx r;
  ASSERT_EQ(parse_rxm_rawx(kRawxPayload, &r), ParseResult::kOk);
  EXPECT_EQ(encode_rxm_rawx(r), kRawxPayload);
}

TEST(RxmRawx, StdevIndexIsTheLowNibble)
{
  Bytes p = kRawxPayload;
  p.at(16 + 27) |= 0xF0;
  RxmRawx r;
  ASSERT_EQ(parse_rxm_rawx(p, &r), ParseResult::kOk);
  EXPECT_EQ(r.meas[0].pr_stdev, 4);
}

TEST(RxmRawx, LengthHandling)
{
  RxmRawx r;
  EXPECT_EQ(parse_rxm_rawx(Bytes(15, 0), &r), ParseResult::kTooShort);
  // header announces two measurements but only one is present
  EXPECT_EQ(parse_rxm_rawx(Bytes(kRawxPayload.begin(), kRawxPayload.end() - 1), &r),
            ParseResult::kTooShort);
  // no measurements
  Bytes empty(kRawxPayload.begin(), kRawxPayload.begin() + 16);
  empty.at(11) = 0;
  ASSERT_EQ(parse_rxm_rawx(empty, &r), ParseResult::kOk);
  EXPECT_TRUE(r.meas.empty());
  EXPECT_EQ(encode_rxm_rawx(r), empty);
}

TEST(RxmSfrbx, RecordedFrame)
{
  RxmSfrbx s;
  ASSERT_EQ(parse_rxm_sfrbx(kSfrbxPayload, &s), ParseResult::kOk);
  EXPECT_EQ(s.gnss_id, 0);
  EXPECT_EQ(s.sv_id, 9);
  EXPECT_EQ(s.sig_id, 0);
  EXPECT_EQ(s.freq_id, 0);
  EXPECT_EQ(s.chn, 155);
  EXPECT_EQ(s.version, 2);
  EXPECT_EQ(s.reserved0, 0x20);
  ASSERT_EQ(s.dwrd.size(), 10u);
  EXPECT_EQ(s.dwrd.front(), 0x22c18609u);
  EXPECT_EQ(s.dwrd.back(), 0x15c82147u);
  EXPECT_EQ(encode_rxm_sfrbx(s), kSfrbxPayload);
}

TEST(RxmSfrbx, LengthHandling)
{
  RxmSfrbx s;
  EXPECT_EQ(parse_rxm_sfrbx(Bytes(7, 0), &s), ParseResult::kTooShort);
  EXPECT_EQ(parse_rxm_sfrbx(Bytes(kSfrbxPayload.begin(), kSfrbxPayload.end() - 1), &s),
            ParseResult::kTooShort);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
