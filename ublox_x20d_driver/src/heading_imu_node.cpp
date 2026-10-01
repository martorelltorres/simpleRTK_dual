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

// Converts ublox_x20d_msgs/NavDAHeading into sensor_msgs/Imu carrying only an
// orientation: yaw from the dual-antenna heading, pitch derived from the baseline
// vector, roll unobserved. Kept separate from the driver so the Imu can be
// regenerated from a bag of nav_daheading without hardware.

#include <cstdint>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ublox_x20d_msgs/NavDAHeading.h>

#include "ublox_x20d_driver/heading_filter.h"

namespace ublox_x20d_driver
{

class HeadingImuNode
{
public:
  HeadingImuNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    heading::FilterParams& p = params_;
    pnh.param("require_gnss_fix_ok", p.require_gnss_fix_ok, p.require_gnss_fix_ok);
    pnh.param("require_rel_pos_heading_valid", p.require_rel_pos_heading_valid, p.require_rel_pos_heading_valid);
    pnh.param("min_carr_soln", p.min_carr_soln, p.min_carr_soln);
    pnh.param("publish_degraded", p.publish_degraded, p.publish_degraded);
    pnh.param("degraded_covariance_factor", p.degraded_covariance_factor, p.degraded_covariance_factor);
    pnh.param("expected_baseline_length_m", p.expected_baseline_length_m, p.expected_baseline_length_m);
    pnh.param("baseline_length_tolerance_m", p.baseline_length_tolerance_m, p.baseline_length_tolerance_m);
    pnh.param("drop_on_baseline_mismatch", p.drop_on_baseline_mismatch, p.drop_on_baseline_mismatch);
    pnh.param("publish_pitch", p.publish_pitch, p.publish_pitch);
    pnh.param("heading_offset_deg", p.heading_offset_deg, p.heading_offset_deg);
    pnh.param("min_heading_std_dev_rad", p.min_heading_std_dev_rad, p.min_heading_std_dev_rad);
    pnh.param("min_pitch_std_dev_rad", p.min_pitch_std_dev_rad, p.min_pitch_std_dev_rad);
    pnh.param<std::string>("frame_id", frame_id_, "");
    if (p.heading_offset_deg != 0.0)
    {
      ROS_WARN("heading_offset_deg = %.2f applied in software; prefer the receiver offset "
               "(receiver_heading_offset_deg in ublox_x20d_node) so recorded nav_daheading already "
               "carries the vehicle heading",
               p.heading_offset_deg);
    }

    imu_pub_ = pnh.advertise<sensor_msgs::Imu>("imu", 10);
    sub_ = nh.subscribe("nav_daheading", 10, &HeadingImuNode::on_heading, this);

    updater_.setHardwareID("ublox_x20d heading");
    updater_.add("heading_imu", this, &HeadingImuNode::diagnose);
    timer_ = pnh.createTimer(ros::Duration(0.5), [this](const ros::TimerEvent&) { updater_.update(); });
  }

private:
  void on_heading(const ublox_x20d_msgs::NavDAHeading::ConstPtr& msg)
  {
    ubx::NavDAHeading m{};
    m.version = msg->version;
    m.itow = msg->itow;
    m.rel_pos_n = msg->rel_pos_n;
    m.rel_pos_e = msg->rel_pos_e;
    m.rel_pos_d = msg->rel_pos_d;
    m.rel_pos_length = msg->rel_pos_length;
    m.rel_pos_heading = msg->rel_pos_heading;
    m.acc_n = msg->acc_n;
    m.acc_e = msg->acc_e;
    m.acc_d = msg->acc_d;
    m.acc_length = msg->acc_length;
    m.acc_heading = msg->acc_heading;
    m.flags = msg->flags;
    m.gnss_fix_ok = msg->gnss_fix_ok;
    m.diff_soln = msg->diff_soln;
    m.rel_pos_valid = msg->rel_pos_valid;
    m.carr_soln = msg->carr_soln;
    m.rel_pos_heading_valid = msg->rel_pos_heading_valid;

    const heading::Orientation o = heading::evaluate(m, params_);
    ++received_;
    last_ = o;
    last_carr_soln_ = m.carr_soln;
    last_acc_heading_deg_ = m.acc_heading * heading::kDAHeadingScaleDeg;
    if (o.baseline_mismatch)
    {
      ROS_WARN_THROTTLE(10.0, "baseline length %.3f m differs from expected %.3f m by more than %.3f m",
                        o.baseline_length_m, params_.expected_baseline_length_m,
                        params_.baseline_length_tolerance_m);
    }

    switch (o.verdict)
    {
      case heading::Verdict::kDropNotValid:
        ++dropped_not_valid_;
        return;
      case heading::Verdict::kDropCarrierSolution:
        ++dropped_carr_soln_;
        return;
      case heading::Verdict::kDropBaseline:
        ++dropped_baseline_;
        return;
      case heading::Verdict::kPublishDegraded:
        ++published_degraded_;
        break;
      case heading::Verdict::kPublish:
        break;
    }

    sensor_msgs::Imu imu;
    imu.header = msg->header;
    if (!frame_id_.empty())
    {
      imu.header.frame_id = frame_id_;
    }
    imu.orientation.x = o.q.x;
    imu.orientation.y = o.q.y;
    imu.orientation.z = o.q.z;
    imu.orientation.w = o.q.w;
    for (size_t i = 0; i < 9; ++i)
    {
      imu.orientation_covariance[i] = o.covariance[i];
    }
    imu.angular_velocity_covariance[0] = -1.0;
    imu.linear_acceleration_covariance[0] = -1.0;
    imu_pub_.publish(imu);
    ++published_;
    last_publish_ = ros::Time::now();
  }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    stat.add("received", received_);
    stat.add("published", published_);
    stat.add("published degraded", published_degraded_);
    stat.add("dropped (not valid)", dropped_not_valid_);
    stat.add("dropped (carrier solution)", dropped_carr_soln_);
    stat.add("dropped (baseline)", dropped_baseline_);
    if (received_ == 0)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::STALE, "no nav_daheading received");
      return;
    }
    stat.add("last carrier solution", static_cast<int>(last_carr_soln_));
    stat.add("last heading accuracy (deg)", last_acc_heading_deg_);
    stat.add("last baseline length (m)", last_.baseline_length_m);
    if (params_.expected_baseline_length_m > 0.0)
    {
      stat.add("expected baseline length (m)", params_.expected_baseline_length_m);
    }

    if (last_.baseline_mismatch)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "baseline length mismatch");
    }
    else if (published_ == 0 || (ros::Time::now() - last_publish_).toSec() > 5.0)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "no recent orientation published");
    }
    else if (last_.verdict == heading::Verdict::kPublishDegraded)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "publishing degraded heading");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "publishing");
    }
  }

  heading::FilterParams params_;
  std::string frame_id_;
  ros::Publisher imu_pub_;
  ros::Subscriber sub_;
  ros::Timer timer_;
  diagnostic_updater::Updater updater_;

  heading::Orientation last_;
  uint8_t last_carr_soln_ = 0;
  double last_acc_heading_deg_ = 0.0;
  ros::Time last_publish_;
  uint64_t received_ = 0;
  uint64_t published_ = 0;
  uint64_t published_degraded_ = 0;
  uint64_t dropped_not_valid_ = 0;
  uint64_t dropped_carr_soln_ = 0;
  uint64_t dropped_baseline_ = 0;
};

}  // namespace ublox_x20d_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "heading_imu");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ublox_x20d_driver::HeadingImuNode node(nh, pnh);
  ros::spin();
  return 0;
}
