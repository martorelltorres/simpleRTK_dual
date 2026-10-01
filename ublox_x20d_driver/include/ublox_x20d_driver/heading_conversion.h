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
// ublox_heading_imu_node/include/ublox_heading_imu_node/heading_conversion.hpp.
// Changes: ROS 1 package layout, pitch derived from the baseline vector,
// pitch variance and quaternion construction added.

#ifndef UBLOX_X20D_DRIVER_HEADING_CONVERSION_H
#define UBLOX_X20D_DRIVER_HEADING_CONVERSION_H

#include <algorithm>
#include <cmath>
#include <cstdint>

// Conversions from UBX-NAV-DAHEADING to a REP-103 orientation. Pure functions,
// no ROS dependencies.
//
// Frames:
// - u-blox reports the GPS1 -> GPS2 baseline in local NED and its heading as the
//   clockwise angle from true north (compass convention).
// - REP-103 uses ENU for the world and FLU (x forward, y left, z up) for the body.
//   Yaw is counter-clockwise from east; positive pitch rotates +x towards -z,
//   i.e. nose down.
//
// The pitch helpers assume the baseline is aligned with the body x axis,
// GPS1 aft and GPS2 forward.
namespace ublox_x20d_driver
{
namespace heading
{

// relPosHeading and accHeading raw units.
constexpr double kDAHeadingScaleDeg = 1e-5;

// Variance reported for an angle the receiver does not observe.
constexpr double kUnobservedVariance = 1e6;

inline double deg_to_rad(double deg)
{
  return deg * M_PI / 180.0;
}

inline double rel_pos_heading_raw_to_deg(int32_t raw)
{
  return raw * kDAHeadingScaleDeg;
}

// Normalize an angle in radians to (-pi, pi].
inline double normalize_angle(double angle_rad)
{
  angle_rad = std::fmod(angle_rad, 2.0 * M_PI);
  if (angle_rad > M_PI)
  {
    angle_rad -= 2.0 * M_PI;
  }
  else if (angle_rad <= -M_PI)
  {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

// NED heading (degrees clockwise from true north) to ENU yaw (radians
// counter-clockwise from east): yaw = pi/2 - heading. offset_deg is added to the
// heading before the conversion.
inline double ned_heading_deg_to_enu_yaw_rad(double heading_deg, double offset_deg = 0.0)
{
  return normalize_angle(M_PI_2 - deg_to_rad(heading_deg + offset_deg));
}

// Yaw variance (rad^2) from the raw accHeading field, with a floor on the
// standard deviation.
inline double heading_acc_to_yaw_variance(uint32_t acc_heading_raw, double min_std_dev_rad)
{
  const double std_dev_rad = std::max(deg_to_rad(acc_heading_raw * kDAHeadingScaleDeg), min_std_dev_rad);
  return std_dev_rad * std_dev_rad;
}

// REP-103 pitch of the baseline: pitch = asin(relPosD / relPosLength), positive
// when GPS2 (forward) is lower than GPS1, i.e. nose down. Returns false, leaving
// pitch_rad untouched, when the length is not positive. The ratio is clamped to
// [-1, 1] because relPosD and relPosLength are rounded independently.
inline bool baseline_pitch_rad(int32_t rel_pos_d_mm, int32_t rel_pos_length_mm, double* pitch_rad)
{
  if (rel_pos_length_mm <= 0)
  {
    return false;
  }
  const double ratio = static_cast<double>(rel_pos_d_mm) / static_cast<double>(rel_pos_length_mm);
  *pitch_rad = std::asin(std::max(-1.0, std::min(1.0, ratio)));
  return true;
}

// Pitch variance (rad^2) from accD: std = accD / (relPosLength * cos(pitch)), with
// a floor on the standard deviation and capped at kUnobservedVariance. Ignores the
// contribution of accLength and its correlation with accD.
inline double pitch_variance(uint32_t acc_d_mm, int32_t rel_pos_length_mm, double pitch_rad,
                             double min_std_dev_rad)
{
  const double denominator = rel_pos_length_mm * std::cos(pitch_rad);
  if (denominator <= 0.0)
  {
    return kUnobservedVariance;
  }
  const double std_dev_rad = std::max(acc_d_mm / denominator, min_std_dev_rad);
  return std::min(std_dev_rad * std_dev_rad, kUnobservedVariance);
}

struct Quaternion
{
  double x;
  double y;
  double z;
  double w;
};

// Quaternion for fixed-axis roll, pitch, yaw applied in that order (equivalently
// intrinsic Z-Y'-X''), the same convention as tf2::Quaternion::setRPY.
inline Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll / 2.0);
  const double sr = std::sin(roll / 2.0);
  const double cp = std::cos(pitch / 2.0);
  const double sp = std::sin(pitch / 2.0);
  const double cy = std::cos(yaw / 2.0);
  const double sy = std::sin(yaw / 2.0);
  return Quaternion{
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  };
}

}  // namespace heading
}  // namespace ublox_x20d_driver

#endif  // UBLOX_X20D_DRIVER_HEADING_CONVERSION_H
