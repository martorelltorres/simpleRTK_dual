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

#ifndef UBLOX_X20D_DRIVER_RAW_LOGGER_H
#define UBLOX_X20D_DRIVER_RAW_LOGGER_H

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>

#include "ublox_x20d_driver/ubx_types.h"

namespace ublox_x20d_driver
{

// Writes complete UBX frames to <dir>/<prefix>_YYYYMMDD_HHMMSS.ubx (UTC), starting a
// new file every `rotate_minutes`. The files are plain UBX streams that RTKLIB reads
// directly (`convbin -r ubx`). No ROS dependencies; not thread-safe.
class RawLogger
{
public:
  enum class Content
  {
    kAll,      // every UBX frame
    kRxmOnly,  // RXM class only (RAWX, SFRBX)
  };

  RawLogger(std::string dir, std::string prefix, double rotate_minutes, Content content);
  ~RawLogger();

  RawLogger(const RawLogger&) = delete;
  RawLogger& operator=(const RawLogger&) = delete;

  // Writes the frame if the content filter accepts it, opening or rotating the file as
  // needed. `now` is the wall-clock time used for the file name and rotation. Returns
  // false on an I/O error; error() then describes it.
  bool write(const ubx::Frame& frame, std::chrono::system_clock::time_point now);

  void close();

  const std::string& current_path() const
  {
    return path_;
  }
  uint64_t bytes_written() const
  {
    return bytes_written_;
  }
  const std::string& error() const
  {
    return error_;
  }

  // "<dir>/<prefix>_YYYYMMDD_HHMMSS.ubx" for `when`, in UTC.
  static std::string file_name(const std::string& dir, const std::string& prefix,
                               std::chrono::system_clock::time_point when);

private:
  bool open(std::chrono::system_clock::time_point now);

  std::string dir_;
  std::string prefix_;
  std::chrono::system_clock::duration rotate_period_;
  Content content_;
  std::FILE* file_ = nullptr;
  std::string path_;
  std::chrono::system_clock::time_point opened_at_;
  uint64_t bytes_written_ = 0;
  std::string error_;
};

}  // namespace ublox_x20d_driver

#endif  // UBLOX_X20D_DRIVER_RAW_LOGGER_H
