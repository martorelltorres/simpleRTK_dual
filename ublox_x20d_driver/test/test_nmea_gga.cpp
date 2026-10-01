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

// Expected sentences, checksums included, were computed independently of this code.

#include <gtest/gtest.h>

#include <string>

#include "ublox_x20d_driver/nmea_gga.h"

using namespace ublox_x20d_driver;
using namespace ublox_x20d_driver::nmea;

namespace
{
ubx::NavPvt rtk_fixed()
{
  ubx::NavPvt p{};
  p.hour = 12;
  p.fix_type = 3;
  p.gnss_fix_ok = true;
  p.carr_soln = ubx::kCarrSolnFixed;
  p.num_sv = 30;
  p.lat = 395712345;  // 39.5712345 N
  p.lon = 26461234;   // 2.6461234 E
  p.p_dop = 123;
  p.hmsl = 1234;
  p.height = 51234;
  return p;
}
}  // namespace

TEST(Gga, RtkFixedNorthEast)
{
  std::string s;
  ASSERT_TRUE(build_gga(rtk_fixed(), &s));
  EXPECT_EQ(s, "$GPGGA,120000.00,3934.27407,N,00238.76740,E,4,30,1.2,1.2,M,50.0,M,,*6C\r\n");
}

TEST(Gga, SouthWestNegativeNanoAndAltitude)
{
  ubx::NavPvt p{};
  p.hour = 0;
  p.nano = -500000000;  // 00:00:00 minus 0.5 s: previous day 23:59:59.50
  p.fix_type = 3;
  p.gnss_fix_ok = true;
  p.num_sv = 7;
  p.lat = -338688000;   // 33.8688 S
  p.lon = -1512093000;  // 151.2093 W
  p.p_dop = 250;
  p.hmsl = -3200;
  p.height = 18600;
  std::string s;
  ASSERT_TRUE(build_gga(p, &s));
  EXPECT_EQ(s, "$GPGGA,235959.50,3352.12800,S,15112.55800,W,1,07,2.5,-3.2,M,21.8,M,,*42\r\n");
}

TEST(Gga, WorstCaseFitsNmeaLimit)
{
  // Longest fields: 89.99999... S, 179.99999... W, 99 satellites, PDOP 100, large heights.
  ubx::NavPvt p = rtk_fixed();
  p.carr_soln = ubx::kCarrSolnFloat;
  p.nano = 990000000;
  p.hour = 23;
  p.min = 59;
  p.sec = 59;
  p.num_sv = 99;
  p.lat = -899999999;
  p.lon = -1799999999;
  p.p_dop = 9999;
  p.hmsl = -9999900;
  p.height = -10999800;
  std::string s;
  ASSERT_TRUE(build_gga(p, &s));
  EXPECT_LE(s.size(), 82u) << s;  // NMEA 0183 maximum, CRLF included
}

TEST(Gga, QualityIndicator)
{
  ubx::NavPvt p = rtk_fixed();
  EXPECT_EQ(gga_quality(p), 4);
  p.carr_soln = ubx::kCarrSolnFloat;
  EXPECT_EQ(gga_quality(p), 5);
  p.carr_soln = ubx::kCarrSolnNone;
  EXPECT_EQ(gga_quality(p), 1);
  p.diff_soln = true;
  EXPECT_EQ(gga_quality(p), 2);
  p.fix_type = 1;
  EXPECT_EQ(gga_quality(p), 6);
}

TEST(Gga, NoSentenceWithoutValidPosition)
{
  const std::string untouched = "unchanged";
  for (int variant = 0; variant < 4; ++variant)
  {
    ubx::NavPvt p = rtk_fixed();
    switch (variant)
    {
      case 0:
        p.fix_type = 0;
        break;
      case 1:
        p.fix_type = 5;  // time only
        break;
      case 2:
        p.gnss_fix_ok = false;
        break;
      case 3:
        p.invalid_llh = true;
        break;
    }
    std::string s = untouched;
    EXPECT_FALSE(build_gga(p, &s)) << variant;
    EXPECT_EQ(s, untouched);
  }
}

TEST(Gga, MinutesNeverPrintSixty)
{
  // 0.99999999 deg is 59.9999994 minutes: must roll over to the next degree.
  EXPECT_EQ(degrees_minutes(0.99999999, 2), "0100.00000");
  EXPECT_EQ(degrees_minutes(-2.5, 3), "00230.00000");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
