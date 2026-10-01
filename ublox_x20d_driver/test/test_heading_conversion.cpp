// Copyright 2026 Australian Robotics Supplies & Technology
// Modifications Copyright 2026 Antoni Martorell
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
//
// Adapted from MonKey-Robotics/ublox_zedx20d,
// ublox_heading_imu_node/test/test_heading_conversion.cpp.
// Changes: pitch, pitch variance, quaternion and end-to-end frame tests added.

#include <gtest/gtest.h>

#include <cmath>

#include "ublox_x20d_driver/heading_conversion.h"

using namespace ublox_x20d_driver::heading;

namespace
{
constexpr double kTol = 1e-9;

struct Vec3
{
  double x;
  double y;
  double z;
};

Vec3 cross(const Vec3& a, const Vec3& b)
{
  return Vec3{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

// Rotate v by the unit quaternion q.
Vec3 rotate(const Quaternion& q, const Vec3& v)
{
  const Vec3 u{ q.x, q.y, q.z };
  const Vec3 c = cross(u, v);
  const Vec3 t{ 2.0 * c.x, 2.0 * c.y, 2.0 * c.z };
  const Vec3 ut = cross(u, t);
  return Vec3{ v.x + q.w * t.x + ut.x, v.y + q.w * t.y + ut.y, v.z + q.w * t.z + ut.z };
}

// Body forward axis (FLU +x) expressed in ENU.
Vec3 forward_axis(double roll, double pitch, double yaw)
{
  return rotate(quaternion_from_rpy(roll, pitch, yaw), Vec3{ 1.0, 0.0, 0.0 });
}
}  // namespace

// --- yaw ---------------------------------------------------------------------

TEST(HeadingToYaw, CardinalNorth)
{
  EXPECT_NEAR(ned_heading_deg_to_enu_yaw_rad(0.0), M_PI_2, kTol);
}

TEST(HeadingToYaw, CardinalEast)
{
  EXPECT_NEAR(ned_heading_deg_to_enu_yaw_rad(90.0), 0.0, kTol);
}

TEST(HeadingToYaw, CardinalSouth)
{
  EXPECT_NEAR(ned_heading_deg_to_enu_yaw_rad(180.0), -M_PI_2, kTol);
}

TEST(HeadingToYaw, CardinalWest)
{
  // -pi and pi are the same direction; only the wrapped magnitude is checked.
  EXPECT_NEAR(std::abs(ned_heading_deg_to_enu_yaw_rad(270.0)), M_PI, kTol);
}

TEST(HeadingToYaw, JustWestOfNorth)
{
  // 359.9 deg is 0.1 deg west of north: yaw 90.1 deg.
  EXPECT_NEAR(ned_heading_deg_to_enu_yaw_rad(359.9), deg_to_rad(90.1), kTol);
}

TEST(HeadingToYaw, RawUnits)
{
  EXPECT_NEAR(rel_pos_heading_raw_to_deg(9000000), 90.0, kTol);
  EXPECT_NEAR(ned_heading_deg_to_enu_yaw_rad(rel_pos_heading_raw_to_deg(9000000)), 0.0, kTol);
}

TEST(HeadingToYaw, OffsetAddedBeforeConversion)
{
  EXPECT_NEAR(ned_heading_deg_to_enu_yaw_rad(0.0, 90.0), 0.0, kTol);
  EXPECT_NEAR(std::abs(ned_heading_deg_to_enu_yaw_rad(0.0, -90.0)), M_PI, kTol);
  EXPECT_NEAR(ned_heading_deg_to_enu_yaw_rad(0.0, 180.0), -M_PI_2, kTol);
}

TEST(HeadingToYaw, OffsetWraps)
{
  // 350 + 90 = 440 = 80 deg -> yaw 10 deg.
  EXPECT_NEAR(ned_heading_deg_to_enu_yaw_rad(350.0, 90.0), deg_to_rad(10.0), kTol);
  // 10 - 90 = -80 = 280 deg -> yaw 170 deg.
  EXPECT_NEAR(ned_heading_deg_to_enu_yaw_rad(10.0, -90.0), deg_to_rad(170.0), kTol);
}

TEST(HeadingToYaw, OutputRange)
{
  for (double heading = -720.0; heading <= 720.0; heading += 7.5)
  {
    const double yaw = ned_heading_deg_to_enu_yaw_rad(heading);
    EXPECT_GT(yaw, -M_PI - kTol);
    EXPECT_LE(yaw, M_PI + kTol);
  }
}

TEST(NormalizeAngle, Boundaries)
{
  EXPECT_NEAR(normalize_angle(M_PI), M_PI, kTol);
  EXPECT_NEAR(normalize_angle(-M_PI), M_PI, kTol);
  EXPECT_NEAR(normalize_angle(3.0 * M_PI), M_PI, kTol);
  EXPECT_NEAR(normalize_angle(-3.0 * M_PI), M_PI, kTol);
  EXPECT_NEAR(normalize_angle(0.0), 0.0, kTol);
  EXPECT_NEAR(normalize_angle(2.5 * M_PI), 0.5 * M_PI, kTol);
}

// --- yaw variance --------------------------------------------------------------

TEST(YawVariance, Scaling)
{
  // accHeading raw 100000 = 1 deg standard deviation.
  const double std_dev = deg_to_rad(1.0);
  EXPECT_NEAR(heading_acc_to_yaw_variance(100000, 0.0), std_dev * std_dev, kTol);
}

TEST(YawVariance, FloorApplies)
{
  EXPECT_NEAR(heading_acc_to_yaw_variance(1, 0.01), 0.01 * 0.01, kTol);
}

// --- pitch -------------------------------------------------------------------

TEST(BaselinePitch, Level)
{
  double pitch = 1.0;
  ASSERT_TRUE(baseline_pitch_rad(0, 1000, &pitch));
  EXPECT_NEAR(pitch, 0.0, kTol);
}

TEST(BaselinePitch, ForwardAntennaLowerIsNoseDownPositive)
{
  // relPosD > 0: GPS2 (forward) below GPS1 -> nose down -> positive REP-103 pitch.
  double pitch = 0.0;
  ASSERT_TRUE(baseline_pitch_rad(100, 1000, &pitch));
  EXPECT_NEAR(pitch, std::asin(0.1), kTol);
  EXPECT_GT(pitch, 0.0);
}

TEST(BaselinePitch, ForwardAntennaHigherIsNoseUpNegative)
{
  double pitch = 0.0;
  ASSERT_TRUE(baseline_pitch_rad(-100, 1000, &pitch));
  EXPECT_NEAR(pitch, -std::asin(0.1), kTol);
  EXPECT_LT(pitch, 0.0);
}

TEST(BaselinePitch, RejectsNonPositiveLength)
{
  double pitch = 0.5;
  EXPECT_FALSE(baseline_pitch_rad(0, 0, &pitch));
  EXPECT_FALSE(baseline_pitch_rad(10, -1000, &pitch));
  EXPECT_EQ(pitch, 0.5);
}

TEST(BaselinePitch, ClampsRoundingOverflow)
{
  // |relPosD| can exceed relPosLength by rounding on a near-vertical baseline.
  double pitch = 0.0;
  ASSERT_TRUE(baseline_pitch_rad(1001, 1000, &pitch));
  EXPECT_NEAR(pitch, M_PI_2, kTol);
  ASSERT_TRUE(baseline_pitch_rad(-1001, 1000, &pitch));
  EXPECT_NEAR(pitch, -M_PI_2, kTol);
}

TEST(PitchVariance, LevelBaseline)
{
  // 10 mm over 1 m -> 0.01 rad.
  EXPECT_NEAR(pitch_variance(10, 1000, 0.0, 0.0), 0.01 * 0.01, kTol);
}

TEST(PitchVariance, GrowsWithPitch)
{
  // cos(60 deg) = 0.5 doubles the standard deviation.
  EXPECT_NEAR(pitch_variance(10, 1000, deg_to_rad(60.0), 0.0), 0.02 * 0.02, kTol);
}

TEST(PitchVariance, FloorApplies)
{
  EXPECT_NEAR(pitch_variance(0, 1000, 0.0, 0.005), 0.005 * 0.005, kTol);
}

TEST(PitchVariance, DegenerateGeometryIsUnobserved)
{
  EXPECT_EQ(pitch_variance(10, 0, 0.0, 0.0), kUnobservedVariance);
  EXPECT_EQ(pitch_variance(10, 1000, M_PI_2, 0.0), kUnobservedVariance);
  EXPECT_EQ(pitch_variance(1000000, 1, 0.0, 0.0), kUnobservedVariance);
}

// --- quaternion ----------------------------------------------------------------

TEST(Quaternion, Identity)
{
  const Quaternion q = quaternion_from_rpy(0.0, 0.0, 0.0);
  EXPECT_NEAR(q.x, 0.0, kTol);
  EXPECT_NEAR(q.y, 0.0, kTol);
  EXPECT_NEAR(q.z, 0.0, kTol);
  EXPECT_NEAR(q.w, 1.0, kTol);
}

TEST(Quaternion, UnitNorm)
{
  const Quaternion q = quaternion_from_rpy(0.3, -0.7, 2.9);
  EXPECT_NEAR(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, kTol);
}

TEST(Quaternion, YawNorthPointsForwardAlongEnuY)
{
  const Vec3 f = forward_axis(0.0, 0.0, ned_heading_deg_to_enu_yaw_rad(0.0));
  EXPECT_NEAR(f.x, 0.0, kTol);
  EXPECT_NEAR(f.y, 1.0, kTol);
  EXPECT_NEAR(f.z, 0.0, kTol);
}

TEST(Quaternion, PositivePitchPointsNoseDown)
{
  const Vec3 f = forward_axis(0.0, deg_to_rad(10.0), 0.0);
  EXPECT_LT(f.z, 0.0);
  EXPECT_NEAR(f.z, -std::sin(deg_to_rad(10.0)), kTol);
}

// --- end to end: NED baseline vector -> ENU body forward axis ---------------------

// For a GPS1 -> GPS2 baseline along the body x axis, the orientation built from
// heading and derived pitch must rotate +x onto the baseline vector expressed in ENU.
class BaselineToOrientation : public ::testing::TestWithParam<Vec3>
{
};

TEST_P(BaselineToOrientation, ForwardAxisMatchesBaseline)
{
  const Vec3 ned = GetParam();
  const double length = std::sqrt(ned.x * ned.x + ned.y * ned.y + ned.z * ned.z);

  double heading_deg = std::atan2(ned.y, ned.x) * 180.0 / M_PI;
  if (heading_deg < 0.0)
  {
    heading_deg += 360.0;
  }
  double pitch = 0.0;
  ASSERT_TRUE(baseline_pitch_rad(static_cast<int32_t>(ned.z), static_cast<int32_t>(length), &pitch));

  const Vec3 f = forward_axis(0.0, pitch, ned_heading_deg_to_enu_yaw_rad(heading_deg));
  EXPECT_NEAR(f.x, ned.y / length, kTol);   // east
  EXPECT_NEAR(f.y, ned.x / length, kTol);   // north
  EXPECT_NEAR(f.z, -ned.z / length, kTol);  // up
}

// NED components in mm, all with an integer length of 1000 mm.
INSTANTIATE_TEST_CASE_P(CardinalAndTilted, BaselineToOrientation,
                        ::testing::Values(Vec3{ 1000, 0, 0 },       // north, level
                                          Vec3{ 0, 1000, 0 },       // east, level
                                          Vec3{ -1000, 0, 0 },      // south, level
                                          Vec3{ 0, -1000, 0 },      // west, level
                                          Vec3{ 600, 800, 0 },      // north-east, level
                                          Vec3{ 960, 0, -280 },     // north, nose up
                                          Vec3{ 0, 960, 280 },      // east, nose down
                                          Vec3{ -480, -640, 600 }   // south-west, nose down
                                          ));

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
