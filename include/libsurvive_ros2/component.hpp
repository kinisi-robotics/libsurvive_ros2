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

  // libsurvive stamps on its own internal epoch (≈0 + seconds-since-init). This
  // offset maps that epoch onto the ROS clock; it is captured exactly once, on
  // the first sample, by get_ros_time(). std::call_once makes the one-time
  // capture safe across the 250 Hz IMU callback and the worker thread.
  std::once_flag epoch_once_;
  rclcpp::Duration epoch_offset_{0, 0};

  // Latest smoothed optical residual ("light_residuals_all") per tracker serial,
  // written by the datalog callback and read by publish_diagnostics(). Guarded
  // because the two run on different threads.
  std::mutex quality_mutex_;
  std::map<std::string, double> light_residuals_;

  bool publish_diagnostics_ = true;
  bool capture_light_residual_ = true;
  double diagnostics_rate_ = 10.0;
  double last_diag_update_s_ = 0.0;
};

}  // namespace libsurvive_ros2

#endif  // LIBSURVIVE_ROS2__COMPONENT_HPP_
