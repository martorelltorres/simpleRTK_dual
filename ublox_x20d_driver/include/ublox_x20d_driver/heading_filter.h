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

#ifndef UBLOX_X20D_DRIVER_HEADING_FILTER_H
#define UBLOX_X20D_DRIVER_HEADING_FILTER_H

#include <algorithm>
#include <array>
#include <cmath>

#include "ublox_x20d_driver/heading_conversion.h"
#include "ublox_x20d_driver/ubx_types.h"

// Decides whether a NAV-DAHEADING epoch becomes a sensor_msgs/Imu orientation and
// computes it. Pure functions, no ROS dependencies.
namespace ublox_x20d_driver
{
namespace heading
{

struct FilterParams
{
  bool require_gnss_fix_ok = true;
  bool require_rel_pos_heading_valid = true;
  int min_carr_soln = ubx::kCarrSolnFixed;  // 0 any, 1 float, 2 fixed
  bool publish_degraded = false;            // below min_carr_soln: publish with inflated covariance
  double degraded_covariance_factor = 100.0;
  double expected_baseline_length_m = 0.0;  // 0 disables the check
  double baseline_length_tolerance_m = 0.05;
  bool drop_on_baseline_mismatch = true;
  bool publish_pitch = true;                // false: yaw-only orientation, pitch unobserved
  double heading_offset_deg = 0.0;          // software offset, added to relPosHeading
  double min_heading_std_dev_rad = 0.001;
  double min_pitch_std_dev_rad = 0.001;
};

enum class Verdict
{
  kPublish,
  kPublishDegraded,
  kDropNotValid,         // gnssFixOK or relPosHeadingValid required and not set
  kDropCarrierSolution,  // carrSoln below min_carr_soln
  kDropBaseline,         // baseline length outside tolerance
};

struct Orientation
{
  Verdict verdict = Verdict::kDropNotValid;
  Quaternion q{ 0.0, 0.0, 0.0, 1.0 };
  double yaw_rad = 0.0;
  double pitch_rad = 0.0;
  std::array<double, 9> covariance{};  // row-major roll, pitch, yaw
  bool baseline_checked = false;       // relPos valid and a check was configured
  bool baseline_mismatch = false;
  double baseline_length_m = 0.0;
};

inline bool published(Verdict v)
{
  return v == Verdict::kPublish || v == Verdict::kPublishDegraded;
}

inline Orientation evaluate(const ubx::NavDAHeading& m, const FilterParams& p)
{
  Orientation out;
  out.baseline_length_m = m.rel_pos_length * 1e-3;

  if ((p.require_gnss_fix_ok && !m.gnss_fix_ok) || (p.require_rel_pos_heading_valid && !m.rel_pos_heading_valid))
  {
    out.verdict = Verdict::kDropNotValid;
    return out;
  }

  if (p.expected_baseline_length_m > 0.0 && m.rel_pos_valid)
  {
    out.baseline_checked = true;
    out.baseline_mismatch =
        std::abs(out.baseline_length_m - p.expected_baseline_length_m) > p.baseline_length_tolerance_m;
    if (out.baseline_mismatch && p.drop_on_baseline_mismatch)
    {
      out.verdict = Verdict::kDropBaseline;
      return out;
    }
  }

  const bool degraded = m.carr_soln < p.min_carr_soln;
  if (degraded && !p.publish_degraded)
  {
    out.verdict = Verdict::kDropCarrierSolution;
    return out;
  }
  out.verdict = degraded ? Verdict::kPublishDegraded : Verdict::kPublish;
  const double factor = degraded ? p.degraded_covariance_factor : 1.0;

  out.yaw_rad = ned_heading_deg_to_enu_yaw_rad(rel_pos_heading_raw_to_deg(m.rel_pos_heading), p.heading_offset_deg);
  double yaw_var = heading_acc_to_yaw_variance(m.acc_heading, p.min_heading_std_dev_rad) * factor;

  double pitch_var = kUnobservedVariance;
  if (p.publish_pitch && m.rel_pos_valid && baseline_pitch_rad(m.rel_pos_d, m.rel_pos_length, &out.pitch_rad))
  {
    pitch_var = std::min(pitch_variance(m.acc_d, m.rel_pos_length, out.pitch_rad, p.min_pitch_std_dev_rad) * factor,
                         kUnobservedVariance);
  }
  else
  {
    out.pitch_rad = 0.0;
  }
  yaw_var = std::min(yaw_var, kUnobservedVariance);

  out.q = quaternion_from_rpy(0.0, out.pitch_rad, out.yaw_rad);
  out.covariance = { kUnobservedVariance, 0.0, 0.0, 0.0, pitch_var, 0.0, 0.0, 0.0, yaw_var };
  return out;
}

}  // namespace heading
}  // namespace ublox_x20d_driver

#endif  // UBLOX_X20D_DRIVER_HEADING_FILTER_H
