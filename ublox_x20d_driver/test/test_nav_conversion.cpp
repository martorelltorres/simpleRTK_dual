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

#include "ublox_x20d_driver/nav_conversion.h"

using namespace ublox_x20d_driver;
using namespace ublox_x20d_driver::nav;

namespace
{
constexpr double kTol = 1e-12;

ubx::NavPvt good_pvt()
{
  ubx::NavPvt p{};
  p.fix_type = 3;
  p.gnss_fix_ok = true;
  return p;
}
}  // namespace

TEST(NavSatStatus, NoFixCases)
{
  for (uint8_t fix_type : { 0, 1, 5 })
  {
    ubx::NavPvt p = good_pvt();
    p.fix_type = fix_type;
    EXPECT_EQ(nav_sat_status(p), kStatusNoFix) << int(fix_type);
  }
  ubx::NavPvt p = good_pvt();
  p.gnss_fix_ok = false;
  EXPECT_EQ(nav_sat_status(p), kStatusNoFix);
  p = good_pvt();
  p.invalid_llh = true;
  EXPECT_EQ(nav_sat_status(p), kStatusNoFix);
}

TEST(NavSatStatus, FixKinds)
{
  ubx::NavPvt p = good_pvt();
  EXPECT_EQ(nav_sat_status(p), kStatusFix);
  p.fix_type = 2;
  EXPECT_EQ(nav_sat_status(p), kStatusFix);
  p.fix_type = 4;
  EXPECT_EQ(nav_sat_status(p), kStatusFix);
  p.diff_soln = true;
  EXPECT_EQ(nav_sat_status(p), kStatusSbasFix);
  p.carr_soln = ubx::kCarrSolnFloat;
  EXPECT_EQ(nav_sat_status(p), kStatusGbasFix);
  p.carr_soln = ubx::kCarrSolnFixed;
  EXPECT_EQ(nav_sat_status(p), kStatusGbasFix);
}

TEST(HighPrecisionPosition, CombinesStandardAndHpParts)
{
  ubx::NavHPPosLLH hp{};
  hp.lat = 395712345;
  hp.lat_hp = 67;
  hp.lon = 26461234;
  hp.lon_hp = -45;
  hp.height = 51234;
  hp.height_hp = -9;
  EXPECT_NEAR(latitude_deg(hp), 39.5712345 + 67e-9, kTol);
  EXPECT_NEAR(longitude_deg(hp), 2.6461234 - 45e-9, kTol);
  EXPECT_NEAR(ellipsoid_height_m(hp), 51.2331, kTol);
}

TEST(PositionCovariance, FromAccuracies)
{
  ubx::NavHPPosLLH hp{};
  hp.h_acc = 140;  // 14 mm
  hp.v_acc = 210;  // 21 mm
  const auto cov = position_covariance(hp);
  EXPECT_NEAR(cov[0], 0.014 * 0.014 / 2.0, kTol);
  EXPECT_NEAR(cov[4], 0.014 * 0.014 / 2.0, kTol);
  EXPECT_NEAR(cov[8], 0.021 * 0.021, kTol);
  for (int i : { 1, 2, 3, 5, 6, 7 })
  {
    EXPECT_EQ(cov[i], 0.0);
  }
}

TEST(Velocity, NedToEnu)
{
  ubx::NavPvt p{};
  p.vel_n = 1000;   // north 1 m/s
  p.vel_e = -2000;  // west 2 m/s
  p.vel_d = 300;    // descending 0.3 m/s
  const Vector3 v = enu_velocity(p);
  EXPECT_NEAR(v.x, -2.0, kTol);
  EXPECT_NEAR(v.y, 1.0, kTol);
  EXPECT_NEAR(v.z, -0.3, kTol);
}

TEST(Velocity, Covariance)
{
  ubx::NavPvt p{};
  p.s_acc = 50;
  const auto cov = twist_covariance(p);
  for (int i = 0; i < 3; ++i)
  {
    EXPECT_NEAR(cov[i * 7], 0.05 * 0.05, kTol);
    EXPECT_EQ(cov[(i + 3) * 7], kUnobservedVariance);
  }
  EXPECT_EQ(cov[1], 0.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
