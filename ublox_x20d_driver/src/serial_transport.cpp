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

#include "ublox_x20d_driver/serial_transport.h"

#include <utility>

#include <boost/asio/buffer.hpp>
#include <boost/asio/write.hpp>
#include <boost/system/system_error.hpp>

namespace ublox_x20d_driver
{

SerialTransport::SerialTransport(DataCallback on_data, ErrorCallback on_error)
  : on_data_(std::move(on_data)), on_error_(std::move(on_error)), port_(io_)
{
}

SerialTransport::~SerialTransport()
{
  close();
}

bool SerialTransport::open(const std::string& device, unsigned int baud_rate, std::string* error)
{
  close();
  boost::system::error_code ec;
  port_.open(device, ec);
  if (!ec)
  {
    port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate), ec);
  }
  if (!ec)
  {
    port_.set_option(boost::asio::serial_port_base::character_size(8), ec);
  }
  if (!ec)
  {
    port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none), ec);
  }
  if (!ec)
  {
    port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one), ec);
  }
  if (!ec)
  {
    port_.set_option(
        boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none), ec);
  }
  if (ec)
  {
    if (error)
    {
      *error = device + ": " + ec.message();
    }
    boost::system::error_code ignored;
    port_.close(ignored);
    return false;
  }

  open_ = true;
  io_.reset();
  start_read();
  thread_ = std::thread([this] { io_.run(); });
  return true;
}

void SerialTransport::close()
{
  open_ = false;
  io_.stop();
  if (thread_.joinable())
  {
    thread_.join();
  }
  boost::system::error_code ignored;
  port_.close(ignored);
}

bool SerialTransport::write(const std::vector<uint8_t>& data, std::string* error)
{
  if (!open_)
  {
    if (error)
    {
      *error = "port not open";
    }
    return false;
  }
  std::lock_guard<std::mutex> lock(write_mutex_);
  boost::system::error_code ec;
  boost::asio::write(port_, boost::asio::buffer(data), ec);
  if (ec)
  {
    if (error)
    {
      *error = ec.message();
    }
    return false;
  }
  return true;
}

void SerialTransport::start_read()
{
  port_.async_read_some(boost::asio::buffer(read_buffer_),
                        [this](const boost::system::error_code& ec, size_t length) {
                          if (ec)
                          {
                            if (open_.exchange(false))
                            {
                              on_error_(ec.message());
                            }
                            return;
                          }
                          if (length > 0)
                          {
                            on_data_(read_buffer_.data(), length);
                          }
                          start_read();
                        });
}

}  // namespace ublox_x20d_driver
