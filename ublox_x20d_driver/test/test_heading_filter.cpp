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

#include <cmath>

#include "ublox_x20d_driver/heading_filter.h"

using namespace ublox_x20d_driver;
using namespace ublox_x20d_driver::heading;

namespace
{
constexpr double kTol = 1e-9;

// A good epoch: 1 m baseline pointing east, level, fixed ambiguities.
ubx::NavDAHeading good()
{
  ubx::NavDAHeading m{};
  m.version = 2;
  m.rel_pos_e = 1000;
  m.rel_pos_length = 1000;
  m.rel_pos_heading = 9000000;  // 90 deg
  m.acc_d = 10;
  m.acc_heading = 50000;  // 0.5 deg
  m.gnss_fix_ok = true;
  m.rel_pos_valid = true;
  m.rel_pos_heading_valid = true;
  m.carr_soln = ubx::kCarrSolnFixed;
  return m;
}
}  // namespace

TEST(HeadingFilter, GoodEpochPublishes)
{
  const Orientation o = evaluate(good(), FilterParams());
  EXPECT_EQ(o.verdict, Verdict::kPublish);
  EXPECT_NEAR(o.yaw_rad, 0.0, kTol);  // east
  EXPECT_NEAR(o.pitch_rad, 0.0, kTol);
  EXPECT_NEAR(o.q.w, 1.0, kTol);
  EXPECT_EQ(o.covariance[0], kUnobservedVariance);
  EXPECT_NEAR(o.covariance[4], 0.01 * 0.01, kTol);
  EXPECT_NEAR(o.covariance[8], std::pow(deg_to_rad(0.5), 2), kTol);
  for (int i : { 1, 2, 3, 5, 6, 7 })
  {
    EXPECT_EQ(o.covariance[i], 0.0);
  }
}

TEST(HeadingFilter, DropsWhenNotValid)
{
  ubx::NavDAHeading m = good();
  m.rel_pos_heading_valid = false;
  EXPECT_EQ(evaluate(m, FilterParams()).verdict, Verdict::kDropNotValid);
  m = good();
  m.gnss_fix_ok = false;
  EXPECT_EQ(evaluate(m, FilterParams()).verdict, Verdict::kDropNotValid);

  FilterParams lax;
  lax.require_gnss_fix_ok = false;
  lax.require_rel_pos_heading_valid = false;
  m.rel_pos_heading_valid = false;
  EXPECT_EQ(evaluate(m, lax).verdict, Verdict::kPublish);
}

TEST(HeadingFilter, CarrierSolutionThreshold)
{
  ubx::NavDAHeading m = good();
  m.carr_soln = ubx::kCarrSolnFloat;
  EXPECT_EQ(evaluate(m, FilterParams()).verdict, Verdict::kDropCarrierSolution);

  FilterParams float_ok;
  float_ok.min_carr_soln = ubx::kCarrSolnFloat;
  EXPECT_EQ(evaluate(m, float_ok).verdict, Verdict::kPublish);

  m.carr_soln = ubx::kCarrSolnNone;
  FilterParams any;
  any.min_carr_soln = 0;
  EXPECT_EQ(evaluate(m, any).verdict, Verdict::kPublish);
}

TEST(HeadingFilter, DegradedInflatesCovariance)
{
  ubx::NavDAHeading m = good();
  const Orientation fixed = evaluate(m, FilterParams());
  m.carr_soln = ubx::kCarrSolnFloat;
  FilterParams p;
  p.publish_degraded = true;
  p.degraded_covariance_factor = 100.0;
  const Orientation degraded = evaluate(m, p);
  EXPECT_EQ(degraded.verdict, Verdict::kPublishDegraded);
  EXPECT_NEAR(degraded.covariance[8], 100.0 * fixed.covariance[8], kTol);
  EXPECT_NEAR(degraded.covariance[4], 100.0 * fixed.covariance[4], kTol);
  EXPECT_EQ(degraded.covariance[0], kUnobservedVariance);
  EXPECT_NEAR(degraded.yaw_rad, fixed.yaw_rad, kTol);
}

TEST(HeadingFilter, BaselineCheck)
{
  FilterParams p;
  p.expected_baseline_length_m = 1.0;
  p.baseline_length_tolerance_m = 0.05;

  ubx::NavDAHeading m = good();
  m.rel_pos_length = 1040;
  Orientation o = evaluate(m, p);
  EXPECT_EQ(o.verdict, Verdict::kPublish);
  EXPECT_TRUE(o.baseline_checked);
  EXPECT_FALSE(o.baseline_mismatch);

  m.rel_pos_length = 1060;
  o = evaluate(m, p);
  EXPECT_EQ(o.verdict, Verdict::kDropBaseline);
  EXPECT_TRUE(o.baseline_mismatch);
  EXPECT_NEAR(o.baseline_length_m, 1.06, kTol);

  p.drop_on_baseline_mismatch = false;
  o = evaluate(m, p);
  EXPECT_EQ(o.verdict, Verdict::kPublish);
  EXPECT_TRUE(o.baseline_mismatch);

  // No check without a valid relative position, nor when disabled.
  m.rel_pos_valid = false;
  EXPECT_FALSE(evaluate(m, p).baseline_checked);
  EXPECT_FALSE(evaluate(good(), FilterParams()).baseline_checked);
}

TEST(HeadingFilter, PitchSignAndYawOnlyMode)
{
  ubx::NavDAHeading m = good();
  m.rel_pos_d = 100;  // forward antenna lower: nose down, positive pitch
  Orientation o = evaluate(m, FilterParams());
  EXPECT_NEAR(o.pitch_rad, std::asin(0.1), kTol);
  EXPECT_LT(o.covariance[4], 1.0);

  FilterParams yaw_only;
  yaw_only.publish_pitch = false;
  o = evaluate(m, yaw_only);
  EXPECT_EQ(o.pitch_rad, 0.0);
  EXPECT_EQ(o.covariance[4], kUnobservedVariance);

  // Without a valid relative position pitch is unobserved even if requested.
  m.rel_pos_valid = false;
  o = evaluate(m, FilterParams());
  EXPECT_EQ(o.pitch_rad, 0.0);
  EXPECT_EQ(o.covariance[4], kUnobservedVariance);
}

TEST(HeadingFilter, SoftwareOffset)
{
  ubx::NavDAHeading m = good();
  m.rel_pos_heading = 0;  // north
  FilterParams p;
  p.heading_offset_deg = 90.0;
  EXPECT_NEAR(evaluate(m, p).yaw_rad, 0.0, kTol);  // reads east
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
