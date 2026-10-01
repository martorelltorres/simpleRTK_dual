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

#include <gtest/gtest.h>

#include <set>
#include <vector>

#include "ublox_x20d_driver/cfg_valset_valget.h"

using namespace ublox_x20d_driver;
using namespace ublox_x20d_driver::cfg;
using Bytes = std::vector<uint8_t>;

namespace
{
KeyValue kv(const CfgKey& key, int64_t value)
{
  return KeyValue{ key.id, encode_value(key.id, value) };
}
}  // namespace

// --- key table -------------------------------------------------------------------

TEST(CfgKeys, SizeFromKeyId)
{
  EXPECT_EQ(value_size(keys::kUsbOutProtUbx.id), 1u);          // L
  EXPECT_EQ(value_size(keys::kMsgOutNavDAHeadingUsb.id), 1u);  // U1
  EXPECT_EQ(value_size(keys::kRateMeas.id), 2u);               // U2
  EXPECT_EQ(value_size(keys::kNavSpgDAHeadingOffset.id), 4u);  // I4
  EXPECT_EQ(value_size(0x50000000u), 8u);
  EXPECT_EQ(value_size(0x00000000u), 0u);
  EXPECT_EQ(value_size(0x60000000u), 0u);
}

TEST(CfgKeys, TypeMatchesSizeCode)
{
  std::set<uint32_t> ids;
  for (const CfgKey& key : keys::kAll)
  {
    EXPECT_TRUE(ids.insert(key.id).second) << key.name << " duplicated";
    switch (key.type)
    {
      case CfgType::kL:
        EXPECT_EQ((key.id >> 28) & 0x7u, 1u) << key.name;
        break;
      case CfgType::kU1:
      case CfgType::kE1:
        EXPECT_EQ((key.id >> 28) & 0x7u, 2u) << key.name;
        break;
      case CfgType::kU2:
        EXPECT_EQ((key.id >> 28) & 0x7u, 3u) << key.name;
        break;
      case CfgType::kI4:
        EXPECT_EQ((key.id >> 28) & 0x7u, 4u) << key.name;
        break;
    }
    EXPECT_EQ(find_key(key.id), &key) << key.name;
  }
  EXPECT_EQ(find_key(0x12345678u), nullptr);
}

TEST(CfgKeys, EncodeDecodeSigned)
{
  const uint64_t raw = encode_value(keys::kNavSpgDAHeadingOffset.id, -9000);
  EXPECT_EQ(raw, 0xFFFFDCD8u);
  EXPECT_EQ(decode_value(keys::kNavSpgDAHeadingOffset, raw), -9000);
  EXPECT_EQ(encode_value(keys::kRateMeas.id, 1000), 1000u);
  EXPECT_EQ(encode_value(keys::kUsbOutProtNmea.id, 0), 0u);
}

// --- VALSET ----------------------------------------------------------------------

TEST(Valset, GoldenFrame)
{
  // CFG_RATE_MEAS = 1000, CFG_NAVSPG_DAHEADING_OFFSET = -9000,
  // CFG_MSGOUT_UBX_NAV_DAHEADING_USB = 1, transactionless, RAM.
  const Bytes expected{ 0xB5, 0x62, 0x06, 0x8A, 0x17, 0x00,
                        0x00, 0x01, 0x00, 0x00,                          // version, layers=RAM, txn, reserved
                        0x01, 0x00, 0x21, 0x30, 0xE8, 0x03,              // CFG_RATE_MEAS
                        0xE4, 0x00, 0x11, 0x40, 0xD8, 0xDC, 0xFF, 0xFF,  // CFG_NAVSPG_DAHEADING_OFFSET
                        0xE2, 0x03, 0x91, 0x20, 0x01,                    // CFG_MSGOUT_UBX_NAV_DAHEADING_USB
                        0x63, 0xA5 };
  const Bytes payload = build_valset_payload({ kv(keys::kRateMeas, 1000), kv(keys::kNavSpgDAHeadingOffset, -9000),
                                               kv(keys::kMsgOutNavDAHeadingUsb, 1) });
  EXPECT_EQ(ubx::encode_frame(ubx::msg_class::kCfg, ubx::msg_id::kCfgValset, payload), expected);
}

TEST(Valset, AlwaysRamOnly)
{
  for (Transaction t : { Transaction::kNone, Transaction::kStart, Transaction::kOngoing, Transaction::kApply })
  {
    const Bytes payload = build_valset_payload({ kv(keys::kRateNav, 1) }, t);
    EXPECT_EQ(payload[1], kValsetLayerMaskRam);
  }
}

TEST(Valset, TransactionHeader)
{
  const Bytes start = build_valset_payload({ kv(keys::kRateNav, 1) }, Transaction::kStart);
  EXPECT_EQ(start[0], 0x01);
  EXPECT_EQ(start[2], 0x01);
  const Bytes apply = build_valset_payload({ kv(keys::kRateNav, 1) }, Transaction::kApply);
  EXPECT_EQ(apply[0], 0x01);
  EXPECT_EQ(apply[2], 0x03);
  const Bytes none = build_valset_payload({ kv(keys::kRateNav, 1) });
  EXPECT_EQ(none[0], 0x00);
  EXPECT_EQ(none[2], 0x00);
}

TEST(Valset, SkipsKeyWithUnknownSize)
{
  const Bytes payload = build_valset_payload({ KeyValue{ 0x60000001u, 1 }, kv(keys::kRateNav, 1) });
  EXPECT_EQ(payload.size(), 4u + 4u + 2u);
}

// --- VALGET ----------------------------------------------------------------------

TEST(Valget, PollPayload)
{
  const Bytes expected{ 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0xE4, 0x00, 0x11, 0x40 };
  EXPECT_EQ(build_valget_payload({ keys::kRateMeas.id, keys::kNavSpgDAHeadingOffset.id }), expected);
  EXPECT_EQ(build_valget_payload({}, kValgetLayerDefault, 3), (Bytes{ 0x00, 0x07, 0x03, 0x00 }));
}

TEST(Valget, ResponseMultiByteValuesNotTruncated)
{
  // Response to the poll above: 1000 must not come back as 232 (its low byte).
  const Bytes payload{ 0x01, 0x00, 0x00, 0x00,                          // version, layer=RAM, position
                       0x01, 0x00, 0x21, 0x30, 0xE8, 0x03,              // CFG_RATE_MEAS = 1000
                       0xE4, 0x00, 0x11, 0x40, 0xD8, 0xDC, 0xFF, 0xFF,  // DAHEADING_OFFSET = -9000
                       0x01, 0x00, 0x78, 0x10, 0x01 };                  // CFG_USBOUTPROT_UBX = 1
  ValgetResponse r;
  ASSERT_TRUE(parse_valget_response(payload, &r));
  EXPECT_EQ(r.version, 0x01);
  EXPECT_EQ(r.layer, kValgetLayerRam);
  ASSERT_EQ(r.values.size(), 3u);
  EXPECT_EQ(r.values[0].key_id, keys::kRateMeas.id);
  EXPECT_EQ(r.values[0].raw, 1000u);
  EXPECT_EQ(r.values[1].key_id, keys::kNavSpgDAHeadingOffset.id);
  EXPECT_EQ(decode_value(keys::kNavSpgDAHeadingOffset, r.values[1].raw), -9000);
  EXPECT_EQ(r.values[1].raw, encode_value(keys::kNavSpgDAHeadingOffset.id, -9000));
  EXPECT_EQ(r.values[2].key_id, keys::kUsbOutProtUbx.id);
  EXPECT_EQ(r.values[2].raw, 1u);
}

TEST(Valget, RoundTripWithValsetEncoding)
{
  // A VALGET response carries the same key/value encoding as a VALSET body.
  const std::vector<KeyValue> values{ kv(keys::kRateMeas, 500), kv(keys::kNavSpgDAHeadingOffset, 18000),
                                      kv(keys::kNavSpgDynModel, kDynModelSea) };
  Bytes payload = build_valset_payload(values);
  payload[0] = 0x01;  // response version
  ValgetResponse r;
  ASSERT_TRUE(parse_valget_response(payload, &r));
  ASSERT_EQ(r.values.size(), values.size());
  for (size_t i = 0; i < values.size(); ++i)
  {
    EXPECT_EQ(r.values[i].key_id, values[i].key_id);
    EXPECT_EQ(r.values[i].raw, values[i].raw);
  }
}

TEST(Valget, EmptyResponse)
{
  ValgetResponse r;
  ASSERT_TRUE(parse_valget_response(Bytes{ 0x01, 0x00, 0x00, 0x00 }, &r));
  EXPECT_TRUE(r.values.empty());
}

TEST(Valget, RejectsMalformed)
{
  ValgetResponse r;
  EXPECT_FALSE(parse_valget_response(Bytes{ 0x01, 0x00, 0x00 }, &r));
  // Truncated value: U2 key with a single value byte.
  EXPECT_FALSE(parse_valget_response(Bytes{ 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0xE8 }, &r));
  // Truncated key id.
  EXPECT_FALSE(parse_valget_response(Bytes{ 0x01, 0x00, 0x00, 0x00, 0x01, 0x00 }, &r));
  // Unknown size code.
  EXPECT_FALSE(parse_valget_response(Bytes{ 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x60, 0x00 }, &r));
}

// --- CFG-RST and ACK ---------------------------------------------------------------

TEST(CfgRst, HotStartControlledSoftwareReset)
{
  EXPECT_EQ(build_cfg_rst_payload(), (Bytes{ 0x00, 0x00, 0x01, 0x00 }));
}

TEST(Ack, Payload)
{
  uint8_t cls = 0;
  uint8_t id = 0;
  ASSERT_TRUE(parse_ack(Bytes{ 0x06, 0x8A }, &cls, &id));
  EXPECT_EQ(cls, ubx::msg_class::kCfg);
  EXPECT_EQ(id, ubx::msg_id::kCfgValset);
  EXPECT_FALSE(parse_ack(Bytes{ 0x06 }, &cls, &id));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
