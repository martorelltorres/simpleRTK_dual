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

#ifndef UBLOX_X20D_DRIVER_CFG_VALSET_VALGET_H
#define UBLOX_X20D_DRIVER_CFG_VALSET_VALGET_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "ublox_x20d_driver/cfg_keys.h"
#include "ublox_x20d_driver/ubx_types.h"

// Payload builders and parsers for UBX-CFG-VALSET, UBX-CFG-VALGET, UBX-CFG-RST and
// UBX-ACK. Pure functions, no ROS dependencies.
//
// Configuration is only ever written to the RAM layer: VALSET takes no layer
// argument. Note the two different layer encodings: VALSET uses a bit mask
// (RAM = 0x01, BBR = 0x02, Flash = 0x04), VALGET an enumeration (RAM = 0, BBR = 1,
// Flash = 2, default = 7).
namespace ublox_x20d_driver
{
namespace cfg
{

constexpr uint8_t kValsetLayerMaskRam = 0x01;
constexpr uint8_t kValgetLayerRam = 0;
constexpr uint8_t kValgetLayerDefault = 7;

// Maximum number of keys in one VALSET or VALGET frame.
constexpr size_t kMaxKeysPerFrame = 64;

struct KeyValue
{
  uint32_t key_id;
  uint64_t raw;  // see encode_value()
};

enum class Transaction : uint8_t
{
  kNone = 0,     // transactionless, applied immediately
  kStart = 1,    // (re)start a transaction
  kOngoing = 2,
  kApply = 3,    // apply and end the transaction
};

// VALSET payload writing `values` to RAM. Version 0x00 for transactionless frames,
// 0x01 when a transaction action is given. Keys with an unknown size code are skipped.
inline std::vector<uint8_t> build_valset_payload(const std::vector<KeyValue>& values,
                                                 Transaction transaction = Transaction::kNone)
{
  std::vector<uint8_t> out;
  ubx::append_u1(out, transaction == Transaction::kNone ? 0x00 : 0x01);
  ubx::append_u1(out, kValsetLayerMaskRam);
  ubx::append_u1(out, static_cast<uint8_t>(transaction));
  ubx::append_u1(out, 0x00);  // reserved
  for (const KeyValue& kv : values)
  {
    const size_t size = value_size(kv.key_id);
    if (size == 0)
    {
      continue;
    }
    ubx::append_u4(out, kv.key_id);
    for (size_t i = 0; i < size; ++i)
    {
      out.push_back(static_cast<uint8_t>(kv.raw >> (8 * i)));
    }
  }
  return out;
}

// VALGET poll payload.
inline std::vector<uint8_t> build_valget_payload(const std::vector<uint32_t>& key_ids,
                                                 uint8_t layer = kValgetLayerRam, uint16_t position = 0)
{
  std::vector<uint8_t> out;
  ubx::append_u1(out, 0x00);  // version
  ubx::append_u1(out, layer);
  ubx::append_u2(out, position);
  for (uint32_t id : key_ids)
  {
    ubx::append_u4(out, id);
  }
  return out;
}

struct ValgetResponse
{
  uint8_t version;
  uint8_t layer;
  uint16_t position;
  std::vector<KeyValue> values;
};

// Parses a VALGET response. Every value is read with all the bytes its key id
// declares. Fails on a truncated pair or a key with an unknown size code, since the
// rest of the payload can then not be delimited.
inline bool parse_valget_response(const std::vector<uint8_t>& payload, ValgetResponse* out)
{
  if (payload.size() < 4)
  {
    return false;
  }
  out->version = ubx::read_u1(&payload[0]);
  out->layer = ubx::read_u1(&payload[1]);
  out->position = ubx::read_u2(&payload[2]);
  out->values.clear();

  size_t pos = 4;
  while (pos < payload.size())
  {
    if (payload.size() - pos < 4)
    {
      return false;
    }
    KeyValue kv{ ubx::read_u4(&payload[pos]), 0 };
    pos += 4;
    const size_t size = value_size(kv.key_id);
    if (size == 0 || payload.size() - pos < size)
    {
      return false;
    }
    for (size_t i = 0; i < size; ++i)
    {
      kv.raw |= static_cast<uint64_t>(payload[pos + i]) << (8 * i);
    }
    pos += size;
    out->values.push_back(kv);
  }
  return true;
}

// UBX-CFG-RST: controlled software reset with hot start. Clears no BBR section and
// writes nothing persistent.
constexpr uint16_t kNavBbrHotStart = 0x0000;
constexpr uint8_t kResetModeControlledSoftware = 0x01;

inline std::vector<uint8_t> build_cfg_rst_payload(uint16_t nav_bbr_mask = kNavBbrHotStart,
                                                  uint8_t reset_mode = kResetModeControlledSoftware)
{
  std::vector<uint8_t> out;
  ubx::append_u2(out, nav_bbr_mask);
  ubx::append_u1(out, reset_mode);
  ubx::append_u1(out, 0x00);  // reserved
  return out;
}

// UBX-ACK-ACK / ACK-NAK payload: class and id of the acknowledged message.
inline bool parse_ack(const std::vector<uint8_t>& payload, uint8_t* msg_class, uint8_t* msg_id)
{
  if (payload.size() < 2)
  {
    return false;
  }
  *msg_class = payload[0];
  *msg_id = payload[1];
  return true;
}

}  // namespace cfg
}  // namespace ublox_x20d_driver

#endif  // UBLOX_X20D_DRIVER_CFG_VALSET_VALGET_H
