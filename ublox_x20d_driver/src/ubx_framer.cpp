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

#include "ublox_x20d_driver/ubx_framer.h"

#include <utility>

namespace ublox_x20d_driver
{
namespace ubx
{

constexpr size_t UbxFramer::kDefaultMaxPayloadLength;

UbxFramer::UbxFramer(FrameCallback callback, size_t max_payload_length)
  : callback_(std::move(callback)), max_payload_length_(max_payload_length)
{
}

void UbxFramer::feed(const uint8_t* data, size_t length)
{
  buffer_.insert(buffer_.end(), data, data + length);

  size_t pos = 0;
  while (true)
  {
    // Find the next sync pair. A trailing lone 0xB5 is kept, its 0x62 may be in
    // the next chunk.
    const size_t search_start = pos;
    while (pos + 1 < buffer_.size() && !(buffer_[pos] == kSync1 && buffer_[pos + 1] == kSync2))
    {
      ++pos;
    }
    if (pos + 1 >= buffer_.size())
    {
      if (pos < buffer_.size() && buffer_[pos] != kSync1)
      {
        ++pos;
      }
      skip(search_start, pos);
      break;
    }
    skip(search_start, pos);

    if (buffer_.size() - pos < kHeaderLength)
    {
      break;
    }
    const size_t payload_length = read_u2(&buffer_[pos + 4]);
    if (payload_length > max_payload_length_)
    {
      ++stats_.length_errors;
      skip(pos, pos + 1);
      ++pos;
      continue;
    }
    const size_t frame_length = kFrameOverhead + payload_length;
    if (buffer_.size() - pos < frame_length)
    {
      break;
    }

    const auto ck = checksum(&buffer_[pos + 2], kHeaderLength - 2 + payload_length);
    const size_t ck_pos = pos + kHeaderLength + payload_length;
    if (ck.first != buffer_[ck_pos] || ck.second != buffer_[ck_pos + 1])
    {
      ++stats_.checksum_errors;
      skip(pos, pos + 1);
      ++pos;
      continue;
    }

    Frame frame;
    frame.msg_class = buffer_[pos + 2];
    frame.msg_id = buffer_[pos + 3];
    frame.payload.assign(buffer_.begin() + pos + kHeaderLength, buffer_.begin() + ck_pos);
    ++stats_.frames;
    pos += frame_length;
    callback_(frame);
  }

  buffer_.erase(buffer_.begin(), buffer_.begin() + pos);
}

void UbxFramer::reset()
{
  buffer_.clear();
  in_nmea_ = false;
  nmea_line_.clear();
}

void UbxFramer::skip(size_t begin, size_t end)
{
  stats_.bytes_skipped += end - begin;
  constexpr size_t kMaxNmeaLength = 120;
  for (size_t i = begin; i < end; ++i)
  {
    const uint8_t c = buffer_[i];
    if (c == '$')
    {
      in_nmea_ = true;
      nmea_line_.assign(1, '$');
    }
    else if (in_nmea_ && c == '\n')
    {
      ++stats_.nmea_sentences;
      in_nmea_ = false;
      last_nmea_ = nmea_line_;
    }
    else if (in_nmea_ && c != '\r' && nmea_line_.size() < kMaxNmeaLength)
    {
      nmea_line_.push_back(static_cast<char>(c));
    }
  }
}

}  // namespace ubx
}  // namespace ublox_x20d_driver
