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

#ifndef UBLOX_X20D_DRIVER_SERIAL_TRANSPORT_H
#define UBLOX_X20D_DRIVER_SERIAL_TRANSPORT_H

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>

namespace ublox_x20d_driver
{

// Serial port with a background read thread. The ZED-X20D USB port is a CDC-ACM
// device (/dev/ttyACM*); the baud rate is accepted but has no effect on it.
//
// The data callback runs on the read thread. The error callback runs once, on the
// read thread, when the port fails (typically because the device disappeared, e.g.
// after UBX-CFG-RST); the transport is then unusable until close() and open().
class SerialTransport
{
public:
  using DataCallback = std::function<void(const uint8_t* data, size_t length)>;
  using ErrorCallback = std::function<void(const std::string& what)>;

  SerialTransport(DataCallback on_data, ErrorCallback on_error);
  ~SerialTransport();

  SerialTransport(const SerialTransport&) = delete;
  SerialTransport& operator=(const SerialTransport&) = delete;

  bool open(const std::string& device, unsigned int baud_rate, std::string* error);

  // Stops the read thread and closes the port. Must not be called from a callback.
  void close();

  bool is_open() const
  {
    return open_;
  }

  // Blocking write of the whole buffer. Safe to call from any thread.
  bool write(const std::vector<uint8_t>& data, std::string* error);

private:
  void start_read();

  DataCallback on_data_;
  ErrorCallback on_error_;
  boost::asio::io_service io_;
  boost::asio::serial_port port_;
  std::thread thread_;
  std::array<uint8_t, 4096> read_buffer_;
  std::mutex write_mutex_;
  std::atomic<bool> open_{ false };
};

}  // namespace ublox_x20d_driver

#endif  // UBLOX_X20D_DRIVER_SERIAL_TRANSPORT_H
