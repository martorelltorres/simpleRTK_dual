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

// Exercises SerialTransport against a pseudo-terminal pair, which behaves as a real
// tty device.

#include <gtest/gtest.h>

#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ublox_x20d_driver/serial_transport.h"

using ublox_x20d_driver::SerialTransport;
using Bytes = std::vector<uint8_t>;

namespace
{
class PtyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char name[256] = {};
    ASSERT_EQ(openpty(&master_, &slave_, name, nullptr, nullptr), 0);
    struct termios tio;
    ASSERT_EQ(tcgetattr(slave_, &tio), 0);
    cfmakeraw(&tio);
    ASSERT_EQ(tcsetattr(slave_, TCSANOW, &tio), 0);
    device_ = name;
  }

  void TearDown() override
  {
    if (master_ >= 0)
    {
      ::close(master_);
    }
    if (slave_ >= 0)
    {
      ::close(slave_);
    }
  }

  // Waits until `received` holds at least n bytes.
  bool wait_for(size_t n)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, std::chrono::seconds(2), [&] { return received_.size() >= n; });
  }

  void on_data(const uint8_t* data, size_t length)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    received_.insert(received_.end(), data, data + length);
    cv_.notify_all();
  }

  int master_ = -1;
  int slave_ = -1;
  std::string device_;
  std::mutex mutex_;
  std::condition_variable cv_;
  Bytes received_;
};
}  // namespace

TEST_F(PtyTest, ReadsWhatTheDeviceSends)
{
  SerialTransport transport([this](const uint8_t* d, size_t n) { on_data(d, n); }, [](const std::string&) {});
  std::string error;
  ASSERT_TRUE(transport.open(device_, 115200, &error)) << error;

  const Bytes sent{ 0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34 };
  ASSERT_EQ(::write(master_, sent.data(), sent.size()), static_cast<ssize_t>(sent.size()));
  ASSERT_TRUE(wait_for(sent.size()));
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(received_, sent);
}

TEST_F(PtyTest, WritesReachTheDevice)
{
  SerialTransport transport([](const uint8_t*, size_t) {}, [](const std::string&) {});
  std::string error;
  ASSERT_TRUE(transport.open(device_, 115200, &error)) << error;

  const Bytes sent{ 0x01, 0x02, 0x03, 0xFF };
  ASSERT_TRUE(transport.write(sent, &error)) << error;
  Bytes got(sent.size());
  size_t total = 0;
  while (total < got.size())
  {
    const ssize_t n = ::read(master_, got.data() + total, got.size() - total);
    ASSERT_GT(n, 0);
    total += static_cast<size_t>(n);
  }
  EXPECT_EQ(got, sent);
}

TEST_F(PtyTest, ReportsLossOfTheDevice)
{
  std::atomic<int> errors{ 0 };
  SerialTransport transport([](const uint8_t*, size_t) {}, [&](const std::string&) { ++errors; });
  std::string error;
  ASSERT_TRUE(transport.open(device_, 115200, &error)) << error;

  // Closing the master side hangs up the slave, as a USB unplug does.
  ::close(master_);
  master_ = -1;
  for (int i = 0; i < 200 && errors == 0; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(errors, 1);
  EXPECT_FALSE(transport.is_open());
  EXPECT_FALSE(transport.write(Bytes{ 0x00 }, &error));
  transport.close();
}

TEST(SerialTransport, OpenFailsOnMissingDevice)
{
  SerialTransport transport([](const uint8_t*, size_t) {}, [](const std::string&) {});
  std::string error;
  EXPECT_FALSE(transport.open("/dev/does-not-exist", 115200, &error));
  EXPECT_NE(error.find("/dev/does-not-exist"), std::string::npos);
  EXPECT_FALSE(transport.is_open());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
