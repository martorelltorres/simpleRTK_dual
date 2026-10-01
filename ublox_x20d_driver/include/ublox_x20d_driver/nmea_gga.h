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

#ifndef UBLOX_X20D_DRIVER_NMEA_GGA_H
#define UBLOX_X20D_DRIVER_NMEA_GGA_H

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "ublox_x20d_driver/ubx_types.h"

// NMEA GGA sentence built from UBX-NAV-PVT, for NTRIP casters that select or synthesise
// the reference station from the rover position. Pure functions, no ROS dependencies.
namespace ublox_x20d_driver
{
namespace nmea
{

// GGA fix quality indicator.
inline int gga_quality(const ubx::NavPvt& pvt)
{
  if (!pvt.gnss_fix_ok || pvt.invalid_llh || pvt.fix_type == 0 || pvt.fix_type == 5)
  {
    return 0;  // invalid
  }
  if (pvt.fix_type == 1)
  {
    return 6;  // dead reckoning only: estimated
  }
  if (pvt.carr_soln == ubx::kCarrSolnFixed)
  {
    return 4;  // RTK fixed
  }
  if (pvt.carr_soln == ubx::kCarrSolnFloat)
  {
    return 5;  // RTK float
  }
  return pvt.diff_soln ? 2 : 1;
}

// "*HH\r\n" terminator for the characters between '$' and '*'.
inline std::string checksum_suffix(const std::string& body)
{
  uint8_t cs = 0;
  for (char c : body)
  {
    cs ^= static_cast<uint8_t>(c);
  }
  char buf[8];
  std::snprintf(buf, sizeof(buf), "*%02X\r\n", cs);
  return buf;
}

// Formats |deg| as degrees and decimal minutes (ddmm.mmmmm, or dddmm.mmmmm with
// deg_digits = 3).
inline std::string degrees_minutes(double deg, int deg_digits)
{
  deg = std::fabs(deg);
  int whole = static_cast<int>(deg);
  double minutes = (deg - whole) * 60.0;
  if (minutes >= 59.999995)  // would print as 60.00000
  {
    ++whole;
    minutes = 0.0;
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%0*d%08.5f", deg_digits, whole, minutes);
  return buf;
}

// $GPGGA sentence (with CRLF) for a NAV-PVT epoch. Returns false, leaving `out`
// untouched, when the epoch has no valid position: sending a zero position would make
// the caster pick the wrong reference station.
inline bool build_gga(const ubx::NavPvt& pvt, std::string* out)
{
  const int quality = gga_quality(pvt);
  if (quality == 0)
  {
    return false;
  }

  // UTC time in centiseconds of the day; nano may be negative.
  constexpr long kDayCs = 86400L * 100L;
  long cs = (pvt.hour * 3600L + pvt.min * 60L + pvt.sec) * 100L + std::lround(pvt.nano / 1e7);
  cs = ((cs % kDayCs) + kDayCs) % kDayCs;
  char time[16];
  std::snprintf(time, sizeof(time), "%02ld%02ld%02ld.%02ld", cs / 360000L, (cs / 6000L) % 60L, (cs / 100L) % 60L,
                cs % 100L);

  const double lat = pvt.lat * 1e-7;
  const double lon = pvt.lon * 1e-7;
  // NAV-PVT carries no HDOP; PDOP, which is never smaller, goes in its place. Heights
  // use one decimal to stay within the 82-character NMEA limit.
  char tail[96];
  std::snprintf(tail, sizeof(tail), "%d,%02d,%.1f,%.1f,M,%.1f,M,,", quality, static_cast<int>(pvt.num_sv),
                pvt.p_dop * 0.01, pvt.hmsl * 1e-3, (pvt.height - pvt.hmsl) * 1e-3);

  const std::string body = std::string("GPGGA,") + time + "," + degrees_minutes(lat, 2) + (lat < 0 ? ",S," : ",N,") +
                           degrees_minutes(lon, 3) + (lon < 0 ? ",W," : ",E,") + tail;
  *out = "$" + body + checksum_suffix(body);
  return true;
}

}  // namespace nmea
}  // namespace ublox_x20d_driver

#endif  // UBLOX_X20D_DRIVER_NMEA_GGA_H
