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

#ifndef UBLOX_X20D_DRIVER_NAV_CONVERSION_H
#define UBLOX_X20D_DRIVER_NAV_CONVERSION_H

#include <array>
#include <cstdint>

#include "ublox_x20d_driver/ubx_types.h"

// Conversions from UBX-NAV-PVT and UBX-NAV-HPPOSLLH to the quantities carried by
// sensor_msgs/NavSatFix and geometry_msgs/TwistWithCovarianceStamped. Pure functions,
// no ROS dependencies.
namespace ublox_x20d_driver
{
namespace nav
{

// Same values as sensor_msgs::NavSatStatus::STATUS_*.
constexpr int8_t kStatusNoFix = -1;
constexpr int8_t kStatusFix = 0;
constexpr int8_t kStatusSbasFix = 1;
constexpr int8_t kStatusGbasFix = 2;

// Variance reported for an unobserved quantity.
constexpr double kUnobservedVariance = 1e6;

// NavSatFix status from NAV-PVT:
// - no fix, time only, dead reckoning only, invalid position or !gnssFixOK -> NO_FIX
// - carrier phase solution (RTK float or fixed)                            -> GBAS_FIX
// - differential corrections without carrier phase                         -> SBAS_FIX
// - otherwise                                                              -> FIX
inline int8_t nav_sat_status(const ubx::NavPvt& pvt)
{
  const bool position_fix = pvt.fix_type == 2 || pvt.fix_type == 3 || pvt.fix_type == 4;
  if (!position_fix || !pvt.gnss_fix_ok || pvt.invalid_llh)
  {
    return kStatusNoFix;
  }
  if (pvt.carr_soln != ubx::kCarrSolnNone)
  {
    return kStatusGbasFix;
  }
  if (pvt.diff_soln)
  {
    return kStatusSbasFix;
  }
  return kStatusFix;
}

inline double latitude_deg(const ubx::NavHPPosLLH& hp)
{
  return hp.lat * 1e-7 + hp.lat_hp * 1e-9;
}

inline double longitude_deg(const ubx::NavHPPosLLH& hp)
{
  return hp.lon * 1e-7 + hp.lon_hp * 1e-9;
}

// Height above the WGS-84 ellipsoid, the altitude NavSatFix expects.
inline double ellipsoid_height_m(const ubx::NavHPPosLLH& hp)
{
  return (hp.height + hp.height_hp * 0.1) * 1e-3;
}

// Row-major ENU position covariance (m^2) from hAcc/vAcc (0.1 mm). hAcc is split
// equally between east and north.
inline std::array<double, 9> position_covariance(const ubx::NavHPPosLLH& hp)
{
  const double h = hp.h_acc * 1e-4;
  const double v = hp.v_acc * 1e-4;
  return { h * h / 2.0, 0.0, 0.0, 0.0, h * h / 2.0, 0.0, 0.0, 0.0, v * v };
}

struct Vector3
{
  double x;
  double y;
  double z;
};

// NED velocity (mm/s) to ENU (m/s).
inline Vector3 enu_velocity(const ubx::NavPvt& pvt)
{
  return { pvt.vel_e * 1e-3, pvt.vel_n * 1e-3, -pvt.vel_d * 1e-3 };
}

// Row-major 6x6 twist covariance: sAcc (mm/s) on each linear axis, angular unobserved.
inline std::array<double, 36> twist_covariance(const ubx::NavPvt& pvt)
{
  const double s = pvt.s_acc * 1e-3;
  std::array<double, 36> cov{};
  for (int i = 0; i < 3; ++i)
  {
    cov[i * 7] = s * s;
    cov[(i + 3) * 7] = kUnobservedVariance;
  }
  return cov;
}

}  // namespace nav
}  // namespace ublox_x20d_driver

#endif  // UBLOX_X20D_DRIVER_NAV_CONVERSION_H
