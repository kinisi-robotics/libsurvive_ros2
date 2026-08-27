// Copyright 2026 Kinisi Robotics
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

#ifndef LIBSURVIVE_ROS2__TRACKING_HEALTH_HPP_
#define LIBSURVIVE_ROS2__TRACKING_HEALTH_HPP_

#include <array>
#include <cstddef>
#include <deque>
#include <limits>
#include <string>

namespace libsurvive_ros2
{

// Optical-tracking health of one tracked object, derived purely from the timing
// of libsurvive's light callbacks so it can be unit-tested without the driver.
//
// Three light events are distinguished, from raw to useful:
//   hit            a sweep pulse decoded from a base station (sweep hook)
//   solved sweep   that pulse turned into a usable angle (sweep_angle hook)
//   integrated     a batch of angles was folded into the pose filter (datalog
//                  "res_error_light_"), i.e. the pose really is light-corrected
// An IMU-only pose keeps arriving at full rate when all light is lost, so the
// pose rate alone cannot tell "tracking" from "dead-reckoning"; light_age_s and
// pose_source can.
class TrackingHealth
{
public:
  static constexpr std::size_t kMaxLighthouses = 16;

  struct Snapshot
  {
    // Sweep hits / solved sweeps per lighthouse index within the window.
    std::array<int, kMaxLighthouses> hits{};
    std::array<int, kMaxLighthouses> solved{};
    // Lighthouses with at least one solved sweep in the window.
    int lighthouses_visible = 0;
    // Seconds since the last solved sweep from any lighthouse; +inf if never.
    double light_age_s = std::numeric_limits<double>::infinity();
    // Seconds since light was last integrated into the pose filter; +inf if never.
    double light_integrated_age_s = std::numeric_limits<double>::infinity();
    // Seconds since the last pose report; +inf if never.
    double pose_age_s = std::numeric_limits<double>::infinity();
    // "light": pose is light-corrected; "imu_only": poses flow but no light is
    // being integrated (dead-reckoning); "none": no recent pose at all.
    std::string pose_source = "none";
  };

  // window_s: how far back hits/solved counts look. imu_only_after_s: light
  // integrated longer ago than this makes the pose IMU-only. pose_stale_after_s:
  // no pose for longer than this makes the source "none".
  explicit TrackingHealth(
    double window_s = 0.5, double imu_only_after_s = 0.25, double pose_stale_after_s = 0.5);

  // All times are seconds on one monotonic clock chosen by the caller.
  void on_hit(int lighthouse, double t);
  void on_solved_sweep(int lighthouse, double t);
  void on_light_integrated(double t);
  void on_pose(double t);

  Snapshot snapshot(double now);

  double window_s() const {return window_s_;}

private:
  void prune(std::deque<double> & q, double now);

  double window_s_;
  double imu_only_after_s_;
  double pose_stale_after_s_;
  std::array<std::deque<double>, kMaxLighthouses> hits_;
  std::array<std::deque<double>, kMaxLighthouses> solved_;
  double last_solved_ = -std::numeric_limits<double>::infinity();
  double last_integrated_ = -std::numeric_limits<double>::infinity();
  double last_pose_ = -std::numeric_limits<double>::infinity();
};

}  // namespace libsurvive_ros2

#endif  // LIBSURVIVE_ROS2__TRACKING_HEALTH_HPP_
