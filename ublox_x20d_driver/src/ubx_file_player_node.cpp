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

// Replays a recorded .ubx file (e.g. one written by ublox_x20d_node with
// enable_raw_observables) and publishes the NAV messages exactly as the driver does,
// so the rest of the pipeline can be run without hardware. Nothing is sent to any
// receiver. Messages are stamped with the current ROS time.

#include <algorithm>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "ublox_x20d_driver/nav_publisher.h"
#include "ublox_x20d_driver/ubx_framer.h"
#include "ublox_x20d_driver/ubx_types.h"

namespace
{
// Pauses longer than this between epochs (gaps in the recording) are shortened to it.
constexpr double kMaxPauseS = 5.0;

bool load_frames(const std::string& path, std::vector<ublox_x20d_driver::ubx::Frame>* frames,
                 ublox_x20d_driver::ubx::UbxFramer::Stats* stats)
{
  std::ifstream in(path, std::ios::binary);
  if (!in)
  {
    return false;
  }
  const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  ublox_x20d_driver::ubx::UbxFramer framer(
      [frames](const ublox_x20d_driver::ubx::Frame& f) { frames->push_back(f); });
  framer.feed(bytes.data(), bytes.size());
  *stats = framer.stats();
  return true;
}
}  // namespace

int main(int argc, char** argv)
{
  using namespace ublox_x20d_driver;
  ros::init(argc, argv, "ubx_file_player");
  ros::NodeHandle pnh("~");

  std::string file;
  std::string frame_id;
  double speed = 1.0;
  double start_delay = 1.0;
  bool loop = false;
  bool publish_gga = false;
  pnh.param<std::string>("file", file, "");
  pnh.param<std::string>("frame_id", frame_id, "gps");
  pnh.param("speed", speed, speed);
  pnh.param("start_delay", start_delay, start_delay);
  pnh.param("loop", loop, loop);
  pnh.param("publish_gga", publish_gga, publish_gga);

  std::vector<ubx::Frame> frames;
  ubx::UbxFramer::Stats stats;
  if (file.empty() || !load_frames(file, &frames, &stats))
  {
    ROS_FATAL("cannot read ~file '%s'", file.c_str());
    return 1;
  }
  ROS_INFO("%s: %zu UBX frames, %llu NMEA sentences, %llu checksum errors", file.c_str(), frames.size(),
           static_cast<unsigned long long>(stats.nmea_sentences),
           static_cast<unsigned long long>(stats.checksum_errors));

  // Raw observables are published whenever the file contains them.
  NavPublisher publisher(pnh, frame_id, publish_gga, true);
  ros::Duration(start_delay).sleep();

  do
  {
    bool have_itow = false;
    uint32_t last_itow = 0;
    size_t published = 0;
    for (const ubx::Frame& frame : frames)
    {
      if (!ros::ok())
      {
        return 0;
      }
      uint32_t itow = 0;
      if (speed > 0.0 && ubx::nav_itow(frame, &itow))
      {
        if (have_itow && itow > last_itow)
        {
          ros::Duration(std::min((itow - last_itow) * 1e-3 / speed, kMaxPauseS)).sleep();
        }
        have_itow = true;
        last_itow = itow;
      }
      const ros::Time now = ros::Time::now();
      if (publisher.handle(frame, now))
      {
        ++published;
      }
      else
      {
        publisher.handle_raw(frame, now);
      }
    }
    ROS_INFO("replayed %zu NAV frames", published);
  } while (loop && ros::ok());

  ros::spin();
  return 0;
}
