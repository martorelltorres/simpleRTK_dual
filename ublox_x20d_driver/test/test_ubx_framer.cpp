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

#include <algorithm>
#include <random>
#include <string>
#include <vector>

#include "ublox_x20d_driver/ubx_framer.h"

using namespace ublox_x20d_driver::ubx;
using Bytes = std::vector<uint8_t>;

namespace
{
Bytes payload_of(size_t length, uint8_t seed)
{
  Bytes payload(length);
  for (size_t i = 0; i < length; ++i)
  {
    payload[i] = static_cast<uint8_t>(seed + 7 * i);
  }
  return payload;
}

Bytes nmea(const std::string& sentence)
{
  return Bytes(sentence.begin(), sentence.end());
}

void append(Bytes& stream, const Bytes& part)
{
  stream.insert(stream.end(), part.begin(), part.end());
}

class FramerTest : public ::testing::Test
{
protected:
  FramerTest() : framer_([this](const Frame& f) { frames_.push_back(f); })
  {
  }

  void feed(const Bytes& bytes)
  {
    framer_.feed(bytes.data(), bytes.size());
  }

  void feed_in_chunks(const Bytes& bytes, const std::vector<size_t>& sizes)
  {
    size_t pos = 0;
    for (size_t i = 0; pos < bytes.size(); ++i)
    {
      const size_t n = std::min(sizes[i % sizes.size()], bytes.size() - pos);
      framer_.feed(bytes.data() + pos, n);
      pos += n;
    }
  }

  static void expect_frame(const Frame& f, uint8_t msg_class, uint8_t msg_id, const Bytes& payload)
  {
    EXPECT_EQ(f.msg_class, msg_class);
    EXPECT_EQ(f.msg_id, msg_id);
    EXPECT_EQ(f.payload, payload);
  }

  UbxFramer framer_;
  std::vector<Frame> frames_;
};

const std::string kThs = "$GNTHS,123.45,A*1C\r\n";
}  // namespace

TEST(UbxChecksum, GoldenMonVerPoll)
{
  // Well-known MON-VER poll frame.
  const Bytes expected{ 0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34 };
  EXPECT_EQ(encode_frame(msg_class::kMon, msg_id::kMonVer, {}), expected);
}

TEST_F(FramerTest, SingleFrame)
{
  const Bytes payload = payload_of(60, 1);
  feed(encode_frame(msg_class::kNav, msg_id::kNavDAHeading, payload));
  ASSERT_EQ(frames_.size(), 1u);
  expect_frame(frames_[0], msg_class::kNav, msg_id::kNavDAHeading, payload);
  EXPECT_EQ(framer_.stats().frames, 1u);
  EXPECT_EQ(framer_.stats().bytes_skipped, 0u);
}

TEST_F(FramerTest, EmptyPayload)
{
  feed(encode_frame(msg_class::kMon, msg_id::kMonVer, {}));
  ASSERT_EQ(frames_.size(), 1u);
  expect_frame(frames_[0], msg_class::kMon, msg_id::kMonVer, {});
}

TEST_F(FramerTest, InterleavedNmea)
{
  const Bytes p1 = payload_of(60, 1);
  const Bytes p2 = payload_of(92, 2);
  Bytes stream = nmea(kThs);
  append(stream, encode_frame(msg_class::kNav, msg_id::kNavDAHeading, p1));
  append(stream, nmea(kThs));
  append(stream, encode_frame(msg_class::kNav, msg_id::kNavPvt, p2));
  append(stream, nmea(kThs));
  feed(stream);

  ASSERT_EQ(frames_.size(), 2u);
  expect_frame(frames_[0], msg_class::kNav, msg_id::kNavDAHeading, p1);
  expect_frame(frames_[1], msg_class::kNav, msg_id::kNavPvt, p2);
  EXPECT_EQ(framer_.stats().nmea_sentences, 3u);
  EXPECT_EQ(framer_.last_nmea(), "$GNTHS,123.45,A*1C");
  EXPECT_EQ(framer_.stats().bytes_skipped, 3 * kThs.size());
  EXPECT_EQ(framer_.stats().checksum_errors, 0u);
}

TEST_F(FramerTest, ArbitraryChunking)
{
  Bytes stream;
  std::vector<Bytes> payloads;
  for (uint8_t i = 0; i < 10; ++i)
  {
    payloads.push_back(payload_of(10 + 17 * i, i));
    append(stream, nmea(kThs));
    append(stream, encode_frame(msg_class::kNav, i, payloads.back()));
  }

  std::mt19937 rng(42);
  std::uniform_int_distribution<size_t> size_dist(1, 40);
  std::vector<size_t> sizes(200);
  std::generate(sizes.begin(), sizes.end(), [&] { return size_dist(rng); });
  feed_in_chunks(stream, sizes);

  ASSERT_EQ(frames_.size(), payloads.size());
  for (size_t i = 0; i < payloads.size(); ++i)
  {
    expect_frame(frames_[i], msg_class::kNav, static_cast<uint8_t>(i), payloads[i]);
  }
  EXPECT_EQ(framer_.stats().nmea_sentences, payloads.size());
}

TEST_F(FramerTest, ByteByByte)
{
  const Bytes payload = payload_of(60, 3);
  Bytes stream = nmea(kThs);
  append(stream, encode_frame(msg_class::kNav, msg_id::kNavDAHeading, payload));
  feed_in_chunks(stream, { 1 });
  ASSERT_EQ(frames_.size(), 1u);
  expect_frame(frames_[0], msg_class::kNav, msg_id::kNavDAHeading, payload);
  EXPECT_EQ(framer_.stats().nmea_sentences, 1u);
}

TEST_F(FramerTest, SyncSplitAcrossChunks)
{
  const Bytes frame = encode_frame(msg_class::kNav, msg_id::kNavPvt, payload_of(92, 4));
  feed(Bytes(frame.begin(), frame.begin() + 1));
  EXPECT_TRUE(frames_.empty());
  feed(Bytes(frame.begin() + 1, frame.end()));
  ASSERT_EQ(frames_.size(), 1u);
  EXPECT_EQ(framer_.stats().bytes_skipped, 0u);
}

TEST_F(FramerTest, BadChecksumDroppedAndNextFrameRecovered)
{
  const Bytes good = payload_of(20, 5);
  Bytes bad = encode_frame(msg_class::kNav, msg_id::kNavDAHeading, payload_of(60, 6));
  bad.back() ^= 0xFF;

  Bytes stream = bad;
  append(stream, encode_frame(msg_class::kNav, msg_id::kNavPvt, good));
  feed(stream);

  ASSERT_EQ(frames_.size(), 1u);
  expect_frame(frames_[0], msg_class::kNav, msg_id::kNavPvt, good);
  EXPECT_EQ(framer_.stats().checksum_errors, 1u);
}

TEST_F(FramerTest, TruncatedFrameFollowedByValidFrame)
{
  // A frame cut short: the header announces 60 bytes but only 20 arrive before the
  // next frame starts. The framer waits for the announced length, fails the checksum
  // and must then find the next frame inside the bytes it already consumed.
  const Bytes truncated_full = encode_frame(msg_class::kNav, msg_id::kNavDAHeading, payload_of(60, 7));
  const Bytes truncated(truncated_full.begin(), truncated_full.begin() + kHeaderLength + 20);
  const Bytes good = payload_of(60, 8);

  Bytes stream = truncated;
  append(stream, encode_frame(msg_class::kNav, msg_id::kNavDAHeading, good));
  feed(stream);

  ASSERT_EQ(frames_.size(), 1u);
  expect_frame(frames_[0], msg_class::kNav, msg_id::kNavDAHeading, good);
  EXPECT_EQ(framer_.stats().checksum_errors, 1u);
}

TEST_F(FramerTest, AbsurdLengthRejectedWithoutWaiting)
{
  Bytes stream{ kSync1, kSync2, msg_class::kNav, msg_id::kNavPvt, 0xFF, 0xFF };
  const Bytes good = payload_of(16, 9);
  append(stream, encode_frame(msg_class::kNav, msg_id::kNavPvt, good));
  feed(stream);

  ASSERT_EQ(frames_.size(), 1u);
  expect_frame(frames_[0], msg_class::kNav, msg_id::kNavPvt, good);
  EXPECT_EQ(framer_.stats().length_errors, 1u);
}

TEST_F(FramerTest, DoublePreamble)
{
  // A stray sync pair right before a genuine frame.
  const Bytes good = payload_of(60, 10);
  Bytes stream{ kSync1, kSync2 };
  append(stream, encode_frame(msg_class::kNav, msg_id::kNavDAHeading, good));
  feed(stream);

  ASSERT_EQ(frames_.size(), 1u);
  expect_frame(frames_[0], msg_class::kNav, msg_id::kNavDAHeading, good);
}

TEST_F(FramerTest, LoneSyncByteIsNoise)
{
  const Bytes good = payload_of(8, 11);
  Bytes stream{ kSync1, 0x00, kSync1, 0x61 };
  append(stream, encode_frame(msg_class::kNav, msg_id::kNavPvt, good));
  feed(stream);

  ASSERT_EQ(frames_.size(), 1u);
  EXPECT_EQ(framer_.stats().bytes_skipped, 4u);
}

TEST_F(FramerTest, FalseSyncInsidePayloadDoesNotSplitFrame)
{
  Bytes payload = payload_of(40, 12);
  payload[10] = kSync1;
  payload[11] = kSync2;
  feed(encode_frame(msg_class::kRxm, msg_id::kRxmRawx, payload));
  ASSERT_EQ(frames_.size(), 1u);
  expect_frame(frames_[0], msg_class::kRxm, msg_id::kRxmRawx, payload);
}

TEST_F(FramerTest, ResetDropsPartialFrame)
{
  const Bytes frame = encode_frame(msg_class::kNav, msg_id::kNavPvt, payload_of(92, 13));
  feed(Bytes(frame.begin(), frame.begin() + 30));
  framer_.reset();
  feed(Bytes(frame.begin() + 30, frame.end()));
  EXPECT_TRUE(frames_.empty());

  feed(frame);
  EXPECT_EQ(frames_.size(), 1u);
}

TEST_F(FramerTest, EncodeFeedRoundTrip)
{
  const Frame original{ msg_class::kCfg, msg_id::kCfgValget, payload_of(33, 14) };
  feed(encode_frame(original));
  ASSERT_EQ(frames_.size(), 1u);
  expect_frame(frames_[0], original.msg_class, original.msg_id, original.payload);
  EXPECT_EQ(encode_frame(frames_[0]), encode_frame(original));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
