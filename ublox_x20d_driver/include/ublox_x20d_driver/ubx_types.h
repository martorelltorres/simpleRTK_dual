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
//
// Message identifiers and payload layouts follow MonKey-Robotics/ublox_zedx20d
// (ublox_dgnss_node/include/ublox_dgnss_node/ubx/), verified on a ZED-X20D with
// HDG 2.00 firmware.

#ifndef UBLOX_X20D_DRIVER_UBX_TYPES_H
#define UBLOX_X20D_DRIVER_UBX_TYPES_H

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

// UBX protocol primitives and payload parsers. Pure functions, no ROS dependencies.
//
// Frame layout: 0xB5 0x62 | class | id | length (U2, LE) | payload | CK_A CK_B.
// The checksum is an 8-bit Fletcher over class, id, length and payload.
namespace ublox_x20d_driver
{
namespace ubx
{

constexpr uint8_t kSync1 = 0xB5;
constexpr uint8_t kSync2 = 0x62;
constexpr size_t kHeaderLength = 6;  // sync (2), class, id, length (2)
constexpr size_t kFrameOverhead = 8;  // header + checksum (2)

namespace msg_class
{
constexpr uint8_t kNav = 0x01;
constexpr uint8_t kRxm = 0x02;
constexpr uint8_t kAck = 0x05;
constexpr uint8_t kCfg = 0x06;
constexpr uint8_t kMon = 0x0A;
}  // namespace msg_class

namespace msg_id
{
constexpr uint8_t kNavPvt = 0x07;
constexpr uint8_t kNavHPPosLLH = 0x14;
constexpr uint8_t kNavDAHeading = 0x45;
constexpr uint8_t kRxmSfrbx = 0x13;
constexpr uint8_t kRxmRawx = 0x15;
constexpr uint8_t kAckNak = 0x00;
constexpr uint8_t kAckAck = 0x01;
constexpr uint8_t kCfgRst = 0x04;
constexpr uint8_t kCfgValset = 0x8A;
constexpr uint8_t kCfgValget = 0x8B;
constexpr uint8_t kMonVer = 0x04;
}  // namespace msg_id

struct Frame
{
  uint8_t msg_class;
  uint8_t msg_id;
  std::vector<uint8_t> payload;
};

// --- little-endian access ------------------------------------------------------

inline uint8_t read_u1(const uint8_t* p)
{
  return p[0];
}

inline uint16_t read_u2(const uint8_t* p)
{
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

inline uint32_t read_u4(const uint8_t* p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

inline uint64_t read_u8(const uint8_t* p)
{
  return static_cast<uint64_t>(read_u4(p)) | (static_cast<uint64_t>(read_u4(p + 4)) << 32);
}

inline int8_t read_i1(const uint8_t* p)
{
  return static_cast<int8_t>(p[0]);
}

inline int16_t read_i2(const uint8_t* p)
{
  return static_cast<int16_t>(read_u2(p));
}

inline int32_t read_i4(const uint8_t* p)
{
  return static_cast<int32_t>(read_u4(p));
}

inline void append_u1(std::vector<uint8_t>& out, uint8_t value)
{
  out.push_back(value);
}

inline void append_u2(std::vector<uint8_t>& out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>(value));
  out.push_back(static_cast<uint8_t>(value >> 8));
}

inline void append_u4(std::vector<uint8_t>& out, uint32_t value)
{
  for (int shift = 0; shift < 32; shift += 8)
  {
    out.push_back(static_cast<uint8_t>(value >> shift));
  }
}

inline void append_u8(std::vector<uint8_t>& out, uint64_t value)
{
  for (int shift = 0; shift < 64; shift += 8)
  {
    out.push_back(static_cast<uint8_t>(value >> shift));
  }
}

// --- framing -------------------------------------------------------------------

// 8-bit Fletcher checksum over `length` bytes, returned as (CK_A, CK_B).
inline std::pair<uint8_t, uint8_t> checksum(const uint8_t* data, size_t length)
{
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  for (size_t i = 0; i < length; ++i)
  {
    ck_a = static_cast<uint8_t>(ck_a + data[i]);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  }
  return { ck_a, ck_b };
}

// Complete frame bytes, sync to checksum.
inline std::vector<uint8_t> encode_frame(uint8_t msg_class, uint8_t msg_id, const std::vector<uint8_t>& payload)
{
  std::vector<uint8_t> out;
  out.reserve(kFrameOverhead + payload.size());
  out.push_back(kSync1);
  out.push_back(kSync2);
  out.push_back(msg_class);
  out.push_back(msg_id);
  append_u2(out, static_cast<uint16_t>(payload.size()));
  out.insert(out.end(), payload.begin(), payload.end());
  const auto ck = checksum(out.data() + 2, out.size() - 2);
  out.push_back(ck.first);
  out.push_back(ck.second);
  return out;
}

inline std::vector<uint8_t> encode_frame(const Frame& frame)
{
  return encode_frame(frame.msg_class, frame.msg_id, frame.payload);
}

// --- payload parsers -----------------------------------------------------------

enum class ParseResult
{
  kOk,
  kTooShort,
  kUnsupportedVersion,
};

constexpr uint8_t kCarrSolnNone = 0;
constexpr uint8_t kCarrSolnFloat = 1;
constexpr uint8_t kCarrSolnFixed = 2;

// UBX-NAV-DAHEADING, payload version 0x02. Version 0x01 (64 bytes, pre-production
// firmware) has a different layout and is rejected.
constexpr uint8_t kNavDAHeadingVersion = 0x02;
constexpr size_t kNavDAHeadingLength = 60;

struct NavDAHeading
{
  uint8_t version;
  uint32_t itow;             // ms
  int32_t rel_pos_n;         // mm
  int32_t rel_pos_e;         // mm
  int32_t rel_pos_d;         // mm
  int32_t rel_pos_length;    // mm
  int32_t rel_pos_heading;   // 1e-5 deg
  uint32_t acc_n;            // mm
  uint32_t acc_e;            // mm
  uint32_t acc_d;            // mm
  uint32_t acc_length;       // mm
  uint32_t acc_heading;      // 1e-5 deg
  uint32_t flags;            // raw
  bool gnss_fix_ok;          // bit 0
  bool diff_soln;            // bit 1
  bool rel_pos_valid;        // bit 2
  uint8_t carr_soln;         // bits 4:3
  bool rel_pos_heading_valid;  // bit 6
};

inline ParseResult parse_nav_daheading(const std::vector<uint8_t>& payload, NavDAHeading* out)
{
  if (payload.empty())
  {
    return ParseResult::kTooShort;
  }
  const uint8_t* p = payload.data();
  if (read_u1(p) != kNavDAHeadingVersion)
  {
    return ParseResult::kUnsupportedVersion;
  }
  if (payload.size() < kNavDAHeadingLength)
  {
    return ParseResult::kTooShort;
  }
  out->version = read_u1(p);
  out->itow = read_u4(p + 4);
  out->rel_pos_n = read_i4(p + 8);
  out->rel_pos_e = read_i4(p + 12);
  out->rel_pos_d = read_i4(p + 16);
  out->rel_pos_length = read_i4(p + 20);
  out->rel_pos_heading = read_i4(p + 24);
  out->acc_n = read_u4(p + 32);
  out->acc_e = read_u4(p + 36);
  out->acc_d = read_u4(p + 40);
  out->acc_length = read_u4(p + 44);
  out->acc_heading = read_u4(p + 48);
  out->flags = read_u4(p + 56);
  out->gnss_fix_ok = (out->flags & 0x01u) != 0;
  out->diff_soln = (out->flags & 0x02u) != 0;
  out->rel_pos_valid = (out->flags & 0x04u) != 0;
  out->carr_soln = static_cast<uint8_t>((out->flags >> 3) & 0x03u);
  out->rel_pos_heading_valid = (out->flags & 0x40u) != 0;
  return ParseResult::kOk;
}

// UBX-NAV-PVT. Longer payloads are accepted and the extra bytes ignored.
constexpr size_t kNavPvtLength = 92;

struct NavPvt
{
  uint32_t itow;  // ms
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  bool valid_date;       // valid bit 0
  bool valid_time;       // valid bit 1
  bool fully_resolved;   // valid bit 2
  uint32_t t_acc;        // ns
  int32_t nano;          // ns
  uint8_t fix_type;
  bool gnss_fix_ok;      // flags bit 0
  bool diff_soln;        // flags bit 1
  uint8_t carr_soln;     // flags bits 7:6
  uint8_t num_sv;
  int32_t lon;           // 1e-7 deg
  int32_t lat;           // 1e-7 deg
  int32_t height;        // mm
  int32_t hmsl;          // mm
  uint32_t h_acc;        // mm
  uint32_t v_acc;        // mm
  int32_t vel_n;         // mm/s
  int32_t vel_e;         // mm/s
  int32_t vel_d;         // mm/s
  int32_t g_speed;       // mm/s
  int32_t head_mot;      // 1e-5 deg
  uint32_t s_acc;        // mm/s
  uint32_t head_acc;     // 1e-5 deg
  uint16_t p_dop;        // 0.01
  bool invalid_llh;      // flags3 bit 0
};

inline ParseResult parse_nav_pvt(const std::vector<uint8_t>& payload, NavPvt* out)
{
  if (payload.size() < kNavPvtLength)
  {
    return ParseResult::kTooShort;
  }
  const uint8_t* p = payload.data();
  out->itow = read_u4(p);
  out->year = read_u2(p + 4);
  out->month = read_u1(p + 6);
  out->day = read_u1(p + 7);
  out->hour = read_u1(p + 8);
  out->min = read_u1(p + 9);
  out->sec = read_u1(p + 10);
  const uint8_t valid = read_u1(p + 11);
  out->valid_date = (valid & 0x01u) != 0;
  out->valid_time = (valid & 0x02u) != 0;
  out->fully_resolved = (valid & 0x04u) != 0;
  out->t_acc = read_u4(p + 12);
  out->nano = read_i4(p + 16);
  out->fix_type = read_u1(p + 20);
  const uint8_t flags = read_u1(p + 21);
  out->gnss_fix_ok = (flags & 0x01u) != 0;
  out->diff_soln = (flags & 0x02u) != 0;
  out->carr_soln = static_cast<uint8_t>((flags >> 6) & 0x03u);
  out->num_sv = read_u1(p + 23);
  out->lon = read_i4(p + 24);
  out->lat = read_i4(p + 28);
  out->height = read_i4(p + 32);
  out->hmsl = read_i4(p + 36);
  out->h_acc = read_u4(p + 40);
  out->v_acc = read_u4(p + 44);
  out->vel_n = read_i4(p + 48);
  out->vel_e = read_i4(p + 52);
  out->vel_d = read_i4(p + 56);
  out->g_speed = read_i4(p + 60);
  out->head_mot = read_i4(p + 64);
  out->s_acc = read_u4(p + 68);
  out->head_acc = read_u4(p + 72);
  out->p_dop = read_u2(p + 76);
  out->invalid_llh = (read_u1(p + 78) & 0x01u) != 0;
  return ParseResult::kOk;
}

// UBX-NAV-HPPOSLLH. Layout of payload version 0x00; the version is reported but
// not enforced.
constexpr uint8_t kNavHPPosLLHVersion = 0x00;
constexpr size_t kNavHPPosLLHLength = 36;

struct NavHPPosLLH
{
  uint8_t version;
  bool invalid_llh;  // flags bit 0
  uint32_t itow;     // ms
  int32_t lon;       // 1e-7 deg
  int32_t lat;       // 1e-7 deg
  int32_t height;    // mm
  int32_t hmsl;      // mm
  int8_t lon_hp;     // 1e-9 deg
  int8_t lat_hp;     // 1e-9 deg
  int8_t height_hp;  // 0.1 mm
  int8_t hmsl_hp;    // 0.1 mm
  uint32_t h_acc;    // 0.1 mm
  uint32_t v_acc;    // 0.1 mm
};

inline ParseResult parse_nav_hpposllh(const std::vector<uint8_t>& payload, NavHPPosLLH* out)
{
  if (payload.size() < kNavHPPosLLHLength)
  {
    return ParseResult::kTooShort;
  }
  const uint8_t* p = payload.data();
  out->version = read_u1(p);
  out->invalid_llh = (read_u1(p + 3) & 0x01u) != 0;
  out->itow = read_u4(p + 4);
  out->lon = read_i4(p + 8);
  out->lat = read_i4(p + 12);
  out->height = read_i4(p + 16);
  out->hmsl = read_i4(p + 20);
  out->lon_hp = read_i1(p + 24);
  out->lat_hp = read_i1(p + 25);
  out->height_hp = read_i1(p + 26);
  out->hmsl_hp = read_i1(p + 27);
  out->h_acc = read_u4(p + 28);
  out->v_acc = read_u4(p + 32);
  return ParseResult::kOk;
}

// UBX-RXM-RAWX: a 16-byte header and 32 bytes per measurement. Bytes 14-15 of the header
// are documented as reserved but carry a value on HDG 2.00 (it advances by 1000 per
// epoch); they are kept so that encode_rxm_rawx() rebuilds the payload exactly.
constexpr size_t kRxmRawxHeaderLength = 16;
constexpr size_t kRxmRawxMeasLength = 32;

struct RawxMeas
{
  double pr_mes;      // m
  double cp_mes;      // cycles
  float do_mes;       // Hz
  uint8_t gnss_id;
  uint8_t sv_id;
  uint8_t sig_id;
  uint8_t freq_id;
  uint16_t locktime;  // ms
  uint8_t cno;        // dBHz
  uint8_t pr_stdev;   // bits 3:0, 0.01 * 2^n m
  uint8_t cp_stdev;   // bits 3:0, 0.004 * n cycles
  uint8_t do_stdev;   // bits 3:0, 0.002 * 2^n Hz
  uint8_t trk_stat;   // raw: bit 0 prValid, 1 cpValid, 2 halfCyc, 3 subHalfCyc
};

struct RxmRawx
{
  double rcv_tow;     // s
  uint16_t week;
  int8_t leap_s;      // s
  uint8_t rec_stat;   // raw: bit 0 leapSec, 1 clkReset
  uint8_t version;
  uint16_t reserved0;
  std::vector<RawxMeas> meas;
};

inline double read_r8(const uint8_t* p)
{
  const uint64_t bits = read_u8(p);
  double value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

inline float read_r4(const uint8_t* p)
{
  const uint32_t bits = read_u4(p);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

inline void append_r8(std::vector<uint8_t>& out, double value)
{
  uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  append_u8(out, bits);
}

inline void append_r4(std::vector<uint8_t>& out, float value)
{
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  append_u4(out, bits);
}

inline ParseResult parse_rxm_rawx(const std::vector<uint8_t>& payload, RxmRawx* out)
{
  if (payload.size() < kRxmRawxHeaderLength)
  {
    return ParseResult::kTooShort;
  }
  const uint8_t* p = payload.data();
  const uint8_t num_meas = read_u1(p + 11);
  if (payload.size() < kRxmRawxHeaderLength + kRxmRawxMeasLength * num_meas)
  {
    return ParseResult::kTooShort;
  }
  out->rcv_tow = read_r8(p);
  out->week = read_u2(p + 8);
  out->leap_s = read_i1(p + 10);
  out->rec_stat = read_u1(p + 12);
  out->version = read_u1(p + 13);
  out->reserved0 = read_u2(p + 14);
  out->meas.resize(num_meas);
  for (size_t i = 0; i < num_meas; ++i)
  {
    const uint8_t* m = p + kRxmRawxHeaderLength + kRxmRawxMeasLength * i;
    RawxMeas& r = out->meas[i];
    r.pr_mes = read_r8(m);
    r.cp_mes = read_r8(m + 8);
    r.do_mes = read_r4(m + 16);
    r.gnss_id = read_u1(m + 20);
    r.sv_id = read_u1(m + 21);
    r.sig_id = read_u1(m + 22);
    r.freq_id = read_u1(m + 23);
    r.locktime = read_u2(m + 24);
    r.cno = read_u1(m + 26);
    r.pr_stdev = read_u1(m + 27) & 0x0Fu;
    r.cp_stdev = read_u1(m + 28) & 0x0Fu;
    r.do_stdev = read_u1(m + 29) & 0x0Fu;
    r.trk_stat = read_u1(m + 30);
  }
  return ParseResult::kOk;
}

// Inverse of parse_rxm_rawx(). The reserved byte of each measurement is written as 0.
inline std::vector<uint8_t> encode_rxm_rawx(const RxmRawx& in)
{
  std::vector<uint8_t> out;
  out.reserve(kRxmRawxHeaderLength + kRxmRawxMeasLength * in.meas.size());
  append_r8(out, in.rcv_tow);
  append_u2(out, in.week);
  append_u1(out, static_cast<uint8_t>(in.leap_s));
  append_u1(out, static_cast<uint8_t>(in.meas.size()));
  append_u1(out, in.rec_stat);
  append_u1(out, in.version);
  append_u2(out, in.reserved0);
  for (const RawxMeas& r : in.meas)
  {
    append_r8(out, r.pr_mes);
    append_r8(out, r.cp_mes);
    append_r4(out, r.do_mes);
    append_u1(out, r.gnss_id);
    append_u1(out, r.sv_id);
    append_u1(out, r.sig_id);
    append_u1(out, r.freq_id);
    append_u2(out, r.locktime);
    append_u1(out, r.cno);
    append_u1(out, r.pr_stdev);
    append_u1(out, r.cp_stdev);
    append_u1(out, r.do_stdev);
    append_u1(out, r.trk_stat);
    append_u1(out, 0);
  }
  return out;
}

// UBX-RXM-SFRBX: an 8-byte header and numWords data words. Byte 7 is documented as
// reserved but is not zero on HDG 2.00; it is kept for an exact rebuild.
constexpr size_t kRxmSfrbxHeaderLength = 8;

struct RxmSfrbx
{
  uint8_t gnss_id;
  uint8_t sv_id;
  uint8_t sig_id;
  uint8_t freq_id;
  uint8_t chn;
  uint8_t version;
  uint8_t reserved0;
  std::vector<uint32_t> dwrd;
};

inline ParseResult parse_rxm_sfrbx(const std::vector<uint8_t>& payload, RxmSfrbx* out)
{
  if (payload.size() < kRxmSfrbxHeaderLength)
  {
    return ParseResult::kTooShort;
  }
  const uint8_t* p = payload.data();
  const uint8_t num_words = read_u1(p + 4);
  if (payload.size() < kRxmSfrbxHeaderLength + 4u * num_words)
  {
    return ParseResult::kTooShort;
  }
  out->gnss_id = read_u1(p);
  out->sv_id = read_u1(p + 1);
  out->sig_id = read_u1(p + 2);
  out->freq_id = read_u1(p + 3);
  out->chn = read_u1(p + 5);
  out->version = read_u1(p + 6);
  out->reserved0 = read_u1(p + 7);
  out->dwrd.resize(num_words);
  for (size_t i = 0; i < num_words; ++i)
  {
    out->dwrd[i] = read_u4(p + kRxmSfrbxHeaderLength + 4 * i);
  }
  return ParseResult::kOk;
}

inline std::vector<uint8_t> encode_rxm_sfrbx(const RxmSfrbx& in)
{
  std::vector<uint8_t> out;
  out.reserve(kRxmSfrbxHeaderLength + 4 * in.dwrd.size());
  append_u1(out, in.gnss_id);
  append_u1(out, in.sv_id);
  append_u1(out, in.sig_id);
  append_u1(out, in.freq_id);
  append_u1(out, static_cast<uint8_t>(in.dwrd.size()));
  append_u1(out, in.chn);
  append_u1(out, in.version);
  append_u1(out, in.reserved0);
  for (uint32_t word : in.dwrd)
  {
    append_u4(out, word);
  }
  return out;
}

// iTOW (ms) of a NAV-PVT, NAV-HPPOSLLH or NAV-DAHEADING frame. False for any other frame
// or a payload too short to hold it.
inline bool nav_itow(const Frame& frame, uint32_t* itow)
{
  if (frame.msg_class != msg_class::kNav)
  {
    return false;
  }
  size_t offset = 0;
  switch (frame.msg_id)
  {
    case msg_id::kNavPvt:
      offset = 0;
      break;
    case msg_id::kNavHPPosLLH:
    case msg_id::kNavDAHeading:
      offset = 4;
      break;
    default:
      return false;
  }
  if (frame.payload.size() < offset + 4)
  {
    return false;
  }
  *itow = read_u4(frame.payload.data() + offset);
  return true;
}

// UBX-MON-VER: swVersion char[30], hwVersion char[10], then any number of char[30]
// extension strings (on the ZED-X20D: "FWVER=HDG 2.00", "PROTVER=57.02", "MOD=ZED-X20D").
struct MonVer
{
  std::string sw_version;
  std::string hw_version;
  std::vector<std::string> extensions;
};

inline std::string fixed_string(const uint8_t* p, size_t max_length)
{
  size_t n = 0;
  while (n < max_length && p[n] != 0)
  {
    ++n;
  }
  return std::string(reinterpret_cast<const char*>(p), n);
}

inline ParseResult parse_mon_ver(const std::vector<uint8_t>& payload, MonVer* out)
{
  if (payload.size() < 40)
  {
    return ParseResult::kTooShort;
  }
  out->sw_version = fixed_string(payload.data(), 30);
  out->hw_version = fixed_string(payload.data() + 30, 10);
  out->extensions.clear();
  for (size_t pos = 40; pos + 30 <= payload.size(); pos += 30)
  {
    out->extensions.push_back(fixed_string(payload.data() + pos, 30));
  }
  return ParseResult::kOk;
}

// Value of the "<key>=" extension, or an empty string.
inline std::string mon_ver_field(const MonVer& ver, const std::string& key)
{
  const std::string prefix = key + "=";
  for (const std::string& ext : ver.extensions)
  {
    if (ext.compare(0, prefix.size(), prefix) == 0)
    {
      return ext.substr(prefix.size());
    }
  }
  return std::string();
}

// True for a ZED-X20D running HDG firmware 2.00 or later. Earlier (pre-production)
// firmware emits NAV-DAHEADING version 0x01, which this driver does not parse.
inline bool mon_ver_compatible(const MonVer& ver)
{
  if (mon_ver_field(ver, "MOD") != "ZED-X20D")
  {
    return false;
  }
  const std::string fw = mon_ver_field(ver, "FWVER");
  const std::string family = "HDG ";
  if (fw.compare(0, family.size(), family) != 0)
  {
    return false;
  }
  const char* version = fw.c_str() + family.size();
  char* end = nullptr;
  const long major = std::strtol(version, &end, 10);
  return end != version && major >= 2;
}

inline std::string mon_ver_summary(const MonVer& ver)
{
  return "MOD=" + mon_ver_field(ver, "MOD") + " FWVER=" + mon_ver_field(ver, "FWVER") +
         " PROTVER=" + mon_ver_field(ver, "PROTVER");
}

}  // namespace ubx
}  // namespace ublox_x20d_driver

#endif  // UBLOX_X20D_DRIVER_UBX_TYPES_H
