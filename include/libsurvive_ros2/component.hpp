// Copyright 2022 Andrew Symington
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef LIBSURVIVE_ROS2__COMPONENT_HPP_
#define LIBSURVIVE_ROS2__COMPONENT_HPP_

#define SURVIVE_ENABLE_FULL_API

// C system
#include <os_generic.h>

// C++ system
#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Other
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "libsurvive/survive_api.h"
#include "libsurvive/survive.h"
#include "libsurvive_ros2/tracking_health.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace libsurvive_ros2
{

class Component : public rclcpp::Node
{
public:
  explicit Component(const rclcpp::NodeOptions & options);
  virtual ~Component();
  rclcpp::Time get_ros_time(const std::string & str, FLT timecode);
  void publish_imu(const sensor_msgs::msg::Imu & msg);
  // Record the latest smoothed optical residual for a tracker serial. Called
  // from the libsurvive datalog callback (libsurvive worker thread).
  void record_light_residual(const std::string & serial, double value);
  // Light-pipeline events for a tracker, from libsurvive's hooks (its threads):
  // a raw sweep decoded from lighthouse `lh`, a sweep solved into an angle, and
  // a batch of light integrated into the pose filter.
  void record_sweep_hit(const std::string & serial, int lh);
  void record_solved_sweep(const std::string & serial, int lh);
  void record_light_integrated(const std::string & serial);

private:
  void work();
  // Publish per-lighthouse calibration flags and per-tracker pose confidence /
  // optical residual on a diagnostic_msgs/DiagnosticArray for downstream gating.
  void publish_diagnostics();

  SurviveSimpleContext * actx_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr cfg_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  std::thread worker_thread_;
  rclcpp::Time last_base_station_update_;
  std::string tracking_frame_;
  double lighthouse_rate_;

  // Latest smoothed optical residual ("light_residuals_all") per tracker serial,
  // written by the datalog callback and read by publish_diagnostics(). Guarded
  // because the two run on different threads.
  std::mutex quality_mutex_;
  std::map<std::string, double> light_residuals_;

  // ROS time each tracker's pose was last broadcast, for pose-freshness in
  // diagnostics. Written and read only on the worker thread, so no lock needed.
  std::map<std::string, rclcpp::Time> last_pose_time_;

  // Optical-tracking health per tracker serial, fed from libsurvive's sweep /
  // sweep-angle / datalog hooks and the pose events, read by publish_diagnostics().
  // Guarded because producers and consumer run on different threads.
  static double mono_now();
  TrackingHealth & health_for(const std::string & serial);
  std::mutex health_mutex_;
  std::map<std::string, TrackingHealth> health_;
  double imu_only_after_s_ = 0.25;
  double pose_stale_after_s_ = 0.5;
  // libsurvive's light-relock self-heal (see patches/), forwarded as driver args.
  double light_relock_timeout_s_ = 3.0;
  double light_relock_force_interval_s_ = -1.0;

  bool publish_diagnostics_ = true;
  bool capture_light_residual_ = true;
  double diagnostics_rate_ = 10.0;
  double last_diag_update_s_ = 0.0;
};

}  // namespace libsurvive_ros2

#endif  // LIBSURVIVE_ROS2__COMPONENT_HPP_
