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

#include "ublox_x20d_driver/raw_logger.h"

#include <cerrno>
#include <cstring>
#include <utility>
#include <vector>

namespace ublox_x20d_driver
{

RawLogger::RawLogger(std::string dir, std::string prefix, double rotate_minutes, Content content)
  : dir_(std::move(dir))
  , prefix_(std::move(prefix))
  , rotate_period_(std::chrono::duration_cast<std::chrono::system_clock::duration>(
        std::chrono::duration<double>(rotate_minutes * 60.0)))
  , content_(content)
{
}

RawLogger::~RawLogger()
{
  close();
}

std::string RawLogger::file_name(const std::string& dir, const std::string& prefix,
                                 std::chrono::system_clock::time_point when)
{
  const std::time_t t = std::chrono::system_clock::to_time_t(when);
  std::tm utc{};
  gmtime_r(&t, &utc);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &utc);
  return dir + "/" + prefix + "_" + stamp + ".ubx";
}

bool RawLogger::open(std::chrono::system_clock::time_point now)
{
  close();
  path_ = file_name(dir_, prefix_, now);
  file_ = std::fopen(path_.c_str(), "ab");
  if (!file_)
  {
    error_ = path_ + ": " + std::strerror(errno);
    return false;
  }
  opened_at_ = now;
  error_.clear();
  return true;
}

bool RawLogger::write(const ubx::Frame& frame, std::chrono::system_clock::time_point now)
{
  if (content_ == Content::kRxmOnly && frame.msg_class != ubx::msg_class::kRxm)
  {
    return true;
  }
  const bool rotate = file_ && rotate_period_.count() > 0 && now - opened_at_ >= rotate_period_;
  if ((!file_ || rotate) && !open(now))
  {
    return false;
  }
  const std::vector<uint8_t> bytes = ubx::encode_frame(frame);
  // Flushed per frame: at ~3 kB/s the cost is negligible and a crash loses nothing.
  if (std::fwrite(bytes.data(), 1, bytes.size(), file_) != bytes.size() || std::fflush(file_) != 0)
  {
    error_ = path_ + ": " + std::strerror(errno);
    close();
    return false;
  }
  bytes_written_ += bytes.size();
  return true;
}

void RawLogger::close()
{
  if (file_)
  {
    std::fclose(file_);
    file_ = nullptr;
  }
}

}  // namespace ublox_x20d_driver
