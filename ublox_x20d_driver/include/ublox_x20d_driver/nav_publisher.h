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

#ifndef UBLOX_X20D_DRIVER_NAV_PUBLISHER_H
#define UBLOX_X20D_DRIVER_NAV_PUBLISHER_H

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include "ublox_x20d_driver/ubx_types.h"

namespace ublox_x20d_driver
{

// Turns UBX NAV frames into ROS messages on the private namespace of `pnh`:
// ~nav_daheading, ~nav_pvt, ~nav_hpposllh (raw), ~fix (NavSatFix, from NAV-HPPOSLLH
// and the NAV-PVT of the same epoch), ~vel (TwistWithCovarianceStamped, with fix) and,
// if enabled, ~nmea (a GGA sentence per NAV-PVT with a valid position, for NTRIP).
// Shared by the driver and the .ubx file player so both publish identically.
//
// handle() is meant to be called from a single thread; snapshot() may be called from
// any thread.
class NavPublisher
{
public:
  struct Snapshot
  {
    bool have_pvt = false;
    ubx::NavPvt pvt{};
    bool have_daheading = false;
    ubx::NavDAHeading daheading{};
    std::chrono::steady_clock::time_point daheading_time;
    uint64_t daheading_count = 0;
  };

  // With publish_raw, ~rxm_rawx and ~rxm_sfrbx are advertised and handle_raw() publishes them.
  NavPublisher(ros::NodeHandle& pnh, const std::string& frame_id, bool publish_gga = false,
               bool publish_raw = false);

  // Publishes the frame if it is NAV-DAHEADING, NAV-PVT or NAV-HPPOSLLH, stamped with
  // `stamp`. Returns true for those three messages, whether or not the payload parsed.
  bool handle(const ubx::Frame& frame, const ros::Time& stamp);

  // Publishes the frame if it is RXM-RAWX or RXM-SFRBX and raw publishing is enabled.
  // Returns true for those two messages, whether or not the payload parsed.
  bool handle_raw(const ubx::Frame& frame, const ros::Time& stamp);

  Snapshot snapshot() const;

private:
  std_msgs::Header header() const;
  void handle_daheading(const ubx::Frame& frame);
  void handle_pvt(const ubx::Frame& frame);
  void handle_hpposllh(const ubx::Frame& frame);
  void handle_rawx(const ubx::Frame& frame);
  void handle_sfrbx(const ubx::Frame& frame);
  void publish_fix_if_paired();

  std::string frame_id_;
  ros::Time stamp_;
  ros::Publisher fix_pub_;
  ros::Publisher vel_pub_;
  ros::Publisher pvt_pub_;
  ros::Publisher hpposllh_pub_;
  ros::Publisher daheading_pub_;
  ros::Publisher nmea_pub_;  // only advertised with publish_gga
  bool publish_raw_ = false;
  ros::Publisher rawx_pub_;   // only advertised with publish_raw
  ros::Publisher sfrbx_pub_;  // only advertised with publish_raw
  bool pvt_length_warned_ = false;
  bool hpposllh_version_warned_ = false;

  mutable std::mutex mutex_;  // guards the members below
  ubx::NavPvt last_pvt_{};
  bool have_pvt_ = false;
  ubx::NavHPPosLLH last_hpposllh_{};
  bool have_hpposllh_ = false;
  uint32_t last_fix_itow_ = UINT32_MAX;
  ubx::NavDAHeading last_daheading_{};
  bool have_daheading_ = false;
  std::chrono::steady_clock::time_point last_daheading_steady_;
  uint64_t daheading_count_ = 0;
};

}  // namespace ublox_x20d_driver

#endif  // UBLOX_X20D_DRIVER_NAV_PUBLISHER_H
