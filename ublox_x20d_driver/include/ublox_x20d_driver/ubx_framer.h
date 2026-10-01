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

#ifndef UBLOX_X20D_DRIVER_UBX_FRAMER_H
#define UBLOX_X20D_DRIVER_UBX_FRAMER_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "ublox_x20d_driver/ubx_types.h"

namespace ublox_x20d_driver
{
namespace ubx
{

// Extracts UBX frames from an arbitrarily chunked byte stream that may also carry
// NMEA sentences or noise.
//
// Bytes are buffered until a complete frame is available. When a candidate frame
// fails (declared length above the limit, or bad checksum), only its first sync
// byte is dropped and the search resumes from the next byte, so a genuine frame
// that starts inside the rejected candidate is still recovered.
class UbxFramer
{
public:
  using FrameCallback = std::function<void(const Frame&)>;

  // RXM-RAWX, the largest message enabled by the driver, is 16 + 32 * numMeas bytes
  // with numMeas an U1.
  static constexpr size_t kDefaultMaxPayloadLength = 8192;

  struct Stats
  {
    uint64_t frames = 0;
    uint64_t checksum_errors = 0;
    uint64_t length_errors = 0;
    uint64_t bytes_skipped = 0;   // bytes outside any valid frame
    uint64_t nmea_sentences = 0;  // '$' ... '\n' sequences among the skipped bytes
  };

  // The callback runs synchronously inside feed() and must not call feed() or reset().
  explicit UbxFramer(FrameCallback callback, size_t max_payload_length = kDefaultMaxPayloadLength);

  void feed(const uint8_t* data, size_t length);

  // Drops any partial frame, keeping the statistics.
  void reset();

  const Stats& stats() const
  {
    return stats_;
  }

  // Last complete NMEA sentence seen among the skipped bytes, without line ending.
  const std::string& last_nmea() const
  {
    return last_nmea_;
  }

private:
  void skip(size_t begin, size_t end);

  FrameCallback callback_;
  size_t max_payload_length_;
  std::vector<uint8_t> buffer_;
  Stats stats_;
  bool in_nmea_ = false;
  std::string nmea_line_;
  std::string last_nmea_;
};

}  // namespace ubx
}  // namespace ublox_x20d_driver

#endif  // UBLOX_X20D_DRIVER_UBX_FRAMER_H
