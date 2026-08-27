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

#include "libsurvive_ros2/tracking_health.hpp"

#include <algorithm>

namespace libsurvive_ros2
{

TrackingHealth::TrackingHealth(
  double window_s, double imu_only_after_s, double pose_stale_after_s)
: window_s_(window_s),
  imu_only_after_s_(imu_only_after_s),
  pose_stale_after_s_(pose_stale_after_s)
{
}

void TrackingHealth::on_hit(int lighthouse, double t)
{
  if (lighthouse < 0 || static_cast<std::size_t>(lighthouse) >= kMaxLighthouses) {
    return;
  }
  hits_[lighthouse].push_back(t);
}

void TrackingHealth::on_solved_sweep(int lighthouse, double t)
{
  if (lighthouse < 0 || static_cast<std::size_t>(lighthouse) >= kMaxLighthouses) {
    return;
  }
  solved_[lighthouse].push_back(t);
  last_solved_ = std::max(last_solved_, t);
}

void TrackingHealth::on_light_integrated(double t)
{
  last_integrated_ = std::max(last_integrated_, t);
}

void TrackingHealth::on_pose(double t)
{
  last_pose_ = std::max(last_pose_, t);
}

void TrackingHealth::prune(std::deque<double> & q, double now)
{
  // Events are appended in time order per source, so trimming the front is
  // enough; a late (out-of-order) stamp only lingers one extra window.
  while (!q.empty() && q.front() < now - window_s_) {
    q.pop_front();
  }
}

TrackingHealth::Snapshot TrackingHealth::snapshot(double now)
{
  Snapshot s;
  for (std::size_t lh = 0; lh < kMaxLighthouses; ++lh) {
    prune(hits_[lh], now);
    prune(solved_[lh], now);
    s.hits[lh] = static_cast<int>(hits_[lh].size());
    s.solved[lh] = static_cast<int>(solved_[lh].size());
    if (s.solved[lh] > 0) {
      ++s.lighthouses_visible;
    }
  }
  s.light_age_s = now - last_solved_;
  s.light_integrated_age_s = now - last_integrated_;
  s.pose_age_s = now - last_pose_;
  if (s.pose_age_s > pose_stale_after_s_) {
    s.pose_source = "none";
  } else if (s.light_integrated_age_s > imu_only_after_s_) {
    s.pose_source = "imu_only";
  } else {
    s.pose_source = "light";
  }
  return s;
}

}  // namespace libsurvive_ros2
