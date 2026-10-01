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

#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "ublox_x20d_driver/raw_logger.h"
#include "ublox_x20d_driver/ubx_framer.h"

using namespace ublox_x20d_driver;
using Clock = std::chrono::system_clock;
using Bytes = std::vector<uint8_t>;

namespace
{
class RawLoggerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char tmpl[] = "/tmp/raw_logger_test_XXXXXX";
    ASSERT_NE(mkdtemp(tmpl), nullptr);
    dir_ = tmpl;
  }

  void TearDown() override
  {
    std::string cmd = "rm -rf '" + dir_ + "'";
    ASSERT_EQ(system(cmd.c_str()), 0);
  }

  static Bytes read_file(const std::string& path)
  {
    std::ifstream in(path, std::ios::binary);
    return Bytes(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  }

  // 2026-09-18 12:00:00 UTC
  static Clock::time_point t0()
  {
    return Clock::from_time_t(1789732800);
  }

  std::string dir_;
};

const ubx::Frame kRawx{ ubx::msg_class::kRxm, ubx::msg_id::kRxmRawx, Bytes(48, 0x11) };
const ubx::Frame kSfrbx{ ubx::msg_class::kRxm, ubx::msg_id::kRxmSfrbx, Bytes(20, 0x22) };
const ubx::Frame kPvt{ ubx::msg_class::kNav, ubx::msg_id::kNavPvt, Bytes(92, 0x33) };
}  // namespace

TEST(RawLoggerName, UtcTimestamp)
{
  EXPECT_EQ(RawLogger::file_name("/data", "x20d", Clock::from_time_t(1789732800)),
            "/data/x20d_20260918_120000.ubx");
}

TEST_F(RawLoggerTest, WritesFramesThatReparse)
{
  RawLogger logger(dir_, "x20d", 60.0, RawLogger::Content::kAll);
  ASSERT_TRUE(logger.write(kRawx, t0())) << logger.error();
  ASSERT_TRUE(logger.write(kPvt, t0()));
  ASSERT_TRUE(logger.write(kSfrbx, t0()));
  EXPECT_EQ(logger.current_path(), dir_ + "/x20d_20260918_120000.ubx");
  logger.close();

  const Bytes contents = read_file(dir_ + "/x20d_20260918_120000.ubx");
  EXPECT_EQ(contents.size(), logger.bytes_written());
  std::vector<ubx::Frame> frames;
  ubx::UbxFramer framer([&](const ubx::Frame& f) { frames.push_back(f); });
  framer.feed(contents.data(), contents.size());
  ASSERT_EQ(frames.size(), 3u);
  EXPECT_EQ(frames[0].msg_id, ubx::msg_id::kRxmRawx);
  EXPECT_EQ(frames[1].msg_id, ubx::msg_id::kNavPvt);
  EXPECT_EQ(frames[2].payload, kSfrbx.payload);
  EXPECT_EQ(framer.stats().bytes_skipped, 0u);
}

TEST_F(RawLoggerTest, RxmOnlyFiltersOtherClasses)
{
  RawLogger logger(dir_, "x20d", 60.0, RawLogger::Content::kRxmOnly);
  ASSERT_TRUE(logger.write(kPvt, t0()));
  EXPECT_EQ(logger.bytes_written(), 0u);
  EXPECT_TRUE(logger.current_path().empty());
  ASSERT_TRUE(logger.write(kRawx, t0()));
  EXPECT_EQ(logger.bytes_written(), ubx::encode_frame(kRawx).size());
}

TEST_F(RawLoggerTest, RotatesAfterPeriod)
{
  RawLogger logger(dir_, "x20d", 1.0, RawLogger::Content::kAll);
  ASSERT_TRUE(logger.write(kRawx, t0()));
  ASSERT_TRUE(logger.write(kRawx, t0() + std::chrono::seconds(59)));
  const std::string first = logger.current_path();
  ASSERT_TRUE(logger.write(kRawx, t0() + std::chrono::seconds(60)));
  const std::string second = logger.current_path();
  EXPECT_NE(first, second);
  EXPECT_EQ(second, dir_ + "/x20d_20260918_120100.ubx");
  logger.close();
  EXPECT_EQ(read_file(first).size(), 2 * ubx::encode_frame(kRawx).size());
  EXPECT_EQ(read_file(second).size(), ubx::encode_frame(kRawx).size());
}

TEST_F(RawLoggerTest, ReportsUnwritableDirectory)
{
  RawLogger logger(dir_ + "/missing", "x20d", 60.0, RawLogger::Content::kAll);
  EXPECT_FALSE(logger.write(kRawx, t0()));
  EXPECT_NE(logger.error().find("missing"), std::string::npos);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
