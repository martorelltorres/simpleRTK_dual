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
// Key ids and types from MonKey-Robotics/ublox_zedx20d,
// ublox_dgnss_node/include/ublox_dgnss_node/ubx/ubx_cfg_item_map.hpp, checked there
// against the u-blox X20 HDG 2.00 Interface Description.

#ifndef UBLOX_X20D_DRIVER_CFG_KEYS_H
#define UBLOX_X20D_DRIVER_CFG_KEYS_H

#include <cstddef>
#include <cstdint>

// Configuration keys written by the driver. Pure definitions, no ROS dependencies.
namespace ublox_x20d_driver
{
namespace cfg
{

enum class CfgType
{
  kL,   // boolean, 1 byte
  kU1,
  kE1,  // enumeration, 1 byte
  kU2,
  kI4,
};

struct CfgKey
{
  const char* name;
  uint32_t id;
  CfgType type;
};

// Value size in bytes, from bits 30:28 of the key id. 0 for an unknown size code.
inline size_t value_size(uint32_t key_id)
{
  switch ((key_id >> 28) & 0x07u)
  {
    case 0x01:  // one bit, stored in one byte
    case 0x02:
      return 1;
    case 0x03:
      return 2;
    case 0x04:
      return 4;
    case 0x05:
      return 8;
    default:
      return 0;
  }
}

// Values travel as the little-endian bytes of the key's size, zero-extended into a
// uint64_t. Comparing encoded values compares exactly what the receiver stores.
inline uint64_t encode_value(uint32_t key_id, int64_t value)
{
  const size_t size = value_size(key_id);
  if (size == 0 || size >= 8)
  {
    return static_cast<uint64_t>(value);
  }
  return static_cast<uint64_t>(value) & ((uint64_t{ 1 } << (8 * size)) - 1);
}

// Inverse of encode_value for display: sign-extends signed types.
inline int64_t decode_value(const CfgKey& key, uint64_t raw)
{
  if (key.type == CfgType::kI4)
  {
    return static_cast<int32_t>(static_cast<uint32_t>(raw));
  }
  return static_cast<int64_t>(raw);
}

namespace keys
{
constexpr CfgKey kUsbInProtUbx{ "CFG_USBINPROT_UBX", 0x10770001, CfgType::kL };
constexpr CfgKey kUsbInProtNmea{ "CFG_USBINPROT_NMEA", 0x10770002, CfgType::kL };
constexpr CfgKey kUsbInProtRtcm3x{ "CFG_USBINPROT_RTCM3X", 0x10770004, CfgType::kL };
constexpr CfgKey kUsbOutProtUbx{ "CFG_USBOUTPROT_UBX", 0x10780001, CfgType::kL };
constexpr CfgKey kUsbOutProtNmea{ "CFG_USBOUTPROT_NMEA", 0x10780002, CfgType::kL };
constexpr CfgKey kUsbOutProtRtcm3x{ "CFG_USBOUTPROT_RTCM3X", 0x10780004, CfgType::kL };
constexpr CfgKey kRateMeas{ "CFG_RATE_MEAS", 0x30210001, CfgType::kU2 };  // ms
constexpr CfgKey kRateNav{ "CFG_RATE_NAV", 0x30210002, CfgType::kU2 };    // measurements per solution
constexpr CfgKey kNavSpgDynModel{ "CFG_NAVSPG_DYNMODEL", 0x20110021, CfgType::kE1 };
constexpr CfgKey kNavSpgDAHeadingOffset{ "CFG_NAVSPG_DAHEADING_OFFSET", 0x401100e4, CfgType::kI4 };  // 0.01 deg
constexpr CfgKey kMsgOutNavDAHeadingUsb{ "CFG_MSGOUT_UBX_NAV_DAHEADING_USB", 0x209103e2, CfgType::kU1 };
constexpr CfgKey kMsgOutNavPvtUsb{ "CFG_MSGOUT_UBX_NAV_PVT_USB", 0x20910009, CfgType::kU1 };
constexpr CfgKey kMsgOutNavHPPosLLHUsb{ "CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB", 0x20910036, CfgType::kU1 };
constexpr CfgKey kMsgOutRxmRawxUsb{ "CFG_MSGOUT_UBX_RXM_RAWX_USB", 0x209102a7, CfgType::kU1 };
constexpr CfgKey kMsgOutRxmSfrbxUsb{ "CFG_MSGOUT_UBX_RXM_SFRBX_USB", 0x20910234, CfgType::kU1 };

constexpr CfgKey kAll[] = {
  kUsbInProtUbx,       kUsbInProtNmea,         kUsbInProtRtcm3x,      kUsbOutProtUbx,
  kUsbOutProtNmea,     kUsbOutProtRtcm3x,      kRateMeas,             kRateNav,
  kNavSpgDynModel,     kNavSpgDAHeadingOffset, kMsgOutNavDAHeadingUsb, kMsgOutNavPvtUsb,
  kMsgOutNavHPPosLLHUsb, kMsgOutRxmRawxUsb,    kMsgOutRxmSfrbxUsb,
};
}  // namespace keys

// CFG_NAVSPG_DYNMODEL value for marine platforms.
constexpr int64_t kDynModelSea = 5;
// CFG_NAVSPG_DAHEADING_OFFSET accepted range, in 0.01 deg.
constexpr int64_t kDAHeadingOffsetLimit = 18000;

// Key definition for an id, or nullptr if the driver does not know it.
inline const CfgKey* find_key(uint32_t id)
{
  for (const CfgKey& key : keys::kAll)
  {
    if (key.id == id)
    {
      return &key;
    }
  }
  return nullptr;
}

}  // namespace cfg
}  // namespace ublox_x20d_driver

#endif  // UBLOX_X20D_DRIVER_CFG_KEYS_H
