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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "libsurvive_ros2/tracking_health.hpp"

using libsurvive_ros2::TrackingHealth;

namespace
{

// Drive the health model the way libsurvive's callbacks would for a tracker in
// full view of `lighthouses` base stations: every 10 ms each lighthouse yields a
// hit and a solved sweep, light is integrated, and a pose is reported.
void run_tracking(TrackingHealth & h, double from, double to, int lighthouses)
{
  for (double t = from; t < to; t += 0.01) {
    for (int lh = 0; lh < lighthouses; ++lh) {
      h.on_hit(lh, t);
      h.on_solved_sweep(lh, t);
    }
    h.on_light_integrated(t);
    h.on_pose(t);
  }
}

// Only the IMU keeps the pose alive: poses continue, no light at all.
void run_imu_only(TrackingHealth & h, double from, double to)
{
  for (double t = from; t < to; t += 0.008) {
    h.on_pose(t);
  }
}

}  // namespace

TEST(TrackingHealth, NothingSeenYet)
{
  TrackingHealth h;
  const auto s = h.snapshot(100.0);
  EXPECT_EQ(s.lighthouses_visible, 0);
  EXPECT_TRUE(std::isinf(s.light_age_s));
  EXPECT_TRUE(std::isinf(s.light_integrated_age_s));
  EXPECT_TRUE(std::isinf(s.pose_age_s));
  EXPECT_EQ(s.pose_source, "none");
}

TEST(TrackingHealth, FullViewIsLightTracked)
{
  TrackingHealth h;
  run_tracking(h, 0.0, 2.0, 4);
  const auto s = h.snapshot(2.0);
  EXPECT_EQ(s.lighthouses_visible, 4);
  for (int lh = 0; lh < 4; ++lh) {
    // 500 ms window at 100 Hz -> 50 events per lighthouse (±1 for boundary).
    EXPECT_NEAR(s.hits[lh], 50, 1);
    EXPECT_NEAR(s.solved[lh], 50, 1);
  }
  EXPECT_EQ(s.hits[4], 0);
  EXPECT_NEAR(s.light_age_s, 0.01, 1e-6);
  EXPECT_NEAR(s.pose_age_s, 0.01, 1e-6);
  EXPECT_EQ(s.pose_source, "light");
}

TEST(TrackingHealth, TotalOcclusionBecomesImuOnlyWithFreshPoses)
{
  // This is the wedge signature: pose_age stays tiny while light_age grows.
  TrackingHealth h;
  run_tracking(h, 0.0, 1.0, 4);
  run_imu_only(h, 1.0, 11.0);
  const auto s = h.snapshot(11.0);
  EXPECT_EQ(s.lighthouses_visible, 0);
  for (int lh = 0; lh < 4; ++lh) {
    EXPECT_EQ(s.hits[lh], 0);
    EXPECT_EQ(s.solved[lh], 0);
  }
  EXPECT_NEAR(s.light_age_s, 10.0, 0.02);
  EXPECT_NEAR(s.light_integrated_age_s, 10.0, 0.02);
  EXPECT_LT(s.pose_age_s, 0.02);
  EXPECT_EQ(s.pose_source, "imu_only");
}

TEST(TrackingHealth, HitsWithoutSolutionsAreNotVisibility)
{
  // Raw sweeps decoded but never turned into angles / integrated: light is
  // arriving, yet the pose is still IMU-only and no lighthouse counts as visible.
  TrackingHealth h;
  run_tracking(h, 0.0, 1.0, 2);
  for (double t = 1.0; t < 3.0; t += 0.01) {
    h.on_hit(0, t);
    h.on_hit(1, t);
    h.on_pose(t);
  }
  const auto s = h.snapshot(3.0);
  EXPECT_NEAR(s.hits[0], 50, 1);
  EXPECT_EQ(s.solved[0], 0);
  EXPECT_EQ(s.lighthouses_visible, 0);
  EXPECT_NEAR(s.light_age_s, 2.0, 0.02);
  EXPECT_EQ(s.pose_source, "imu_only");
}

TEST(TrackingHealth, PartialOcclusionCountsOnlyVisibleLighthouses)
{
  TrackingHealth h;
  run_tracking(h, 0.0, 1.0, 4);
  // Lighthouses 2 and 3 disappear; 0 and 1 keep the solution alive.
  for (double t = 1.0; t < 2.0; t += 0.01) {
    for (int lh = 0; lh < 2; ++lh) {
      h.on_hit(lh, t);
      h.on_solved_sweep(lh, t);
    }
    h.on_light_integrated(t);
    h.on_pose(t);
  }
  const auto s = h.snapshot(2.0);
  EXPECT_EQ(s.lighthouses_visible, 2);
  EXPECT_GT(s.solved[0], 0);
  EXPECT_GT(s.solved[1], 0);
  EXPECT_EQ(s.solved[2], 0);
  EXPECT_EQ(s.solved[3], 0);
  EXPECT_EQ(s.pose_source, "light");
}

TEST(TrackingHealth, RecoveryAfterOcclusion)
{
  TrackingHealth h;
  run_tracking(h, 0.0, 1.0, 4);
  run_imu_only(h, 1.0, 20.0);
  EXPECT_EQ(h.snapshot(20.0).pose_source, "imu_only");
  run_tracking(h, 20.0, 21.0, 4);
  const auto s = h.snapshot(21.0);
  EXPECT_EQ(s.lighthouses_visible, 4);
  EXPECT_EQ(s.pose_source, "light");
  EXPECT_LT(s.light_age_s, 0.02);
}

TEST(TrackingHealth, StalePoseIsNone)
{
  TrackingHealth h;
  run_tracking(h, 0.0, 1.0, 4);
  const auto s = h.snapshot(2.0);
  EXPECT_EQ(s.pose_source, "none");
  EXPECT_NEAR(s.pose_age_s, 1.0, 0.02);
  EXPECT_EQ(s.lighthouses_visible, 0);
}

TEST(TrackingHealth, IgnoresOutOfRangeLighthouseIndex)
{
  TrackingHealth h;
  h.on_hit(-1, 0.0);
  h.on_hit(static_cast<int>(TrackingHealth::kMaxLighthouses), 0.0);
  h.on_solved_sweep(99, 0.0);
  const auto s = h.snapshot(0.1);
  EXPECT_EQ(s.lighthouses_visible, 0);
  EXPECT_TRUE(std::isinf(s.light_age_s));
}

TEST(TrackingHealth, WindowIsConfigurable)
{
  TrackingHealth h(1.0);
  run_tracking(h, 0.0, 2.0, 1);
  EXPECT_NEAR(h.snapshot(2.0).hits[0], 100, 1);
}

TEST(WedgeRestart, RequiresLongDarknessAndFailedRelocks)
{
  using libsurvive_ros2::wedge_restart_due;
  EXPECT_TRUE(wedge_restart_due(25.0, 5, 20.0, 2));
  EXPECT_FALSE(wedge_restart_due(19.9, 5, 20.0, 2)) << "not dark long enough";
  EXPECT_FALSE(wedge_restart_due(25.0, 1, 20.0, 2)) << "mode switch not yet retried enough";
  EXPECT_TRUE(wedge_restart_due(25.0, 2, 20.0, 2)) << "min_relocks is inclusive";
  EXPECT_FALSE(wedge_restart_due(25.0, 5, 0.0, 2)) << "after_s <= 0 disables";
  EXPECT_FALSE(wedge_restart_due(25.0, 5, -1.0, 2));
  EXPECT_FALSE(wedge_restart_due(std::numeric_limits<double>::infinity(), 5, 20.0, 2))
    << "never had light: start-up, not a wedge";
  EXPECT_TRUE(wedge_restart_due(0.6, 0, 0.5, 0)) << "zero min_relocks allowed";
}

TEST(StampRecordPath, OnlyRewritesTheRecordArgument)
{
  using libsurvive_ros2::stamp_record_path;
  EXPECT_EQ(
    stamp_record_path("--disable-calibrate 1 --record /tmp/a.rec --light-relock-timeout 3", "T"),
    "--disable-calibrate 1 --record /tmp/a.rec.T --light-relock-timeout 3");
  EXPECT_EQ(stamp_record_path("--record /tmp/a.rec", "T"), "--record /tmp/a.rec.T");
  EXPECT_EQ(stamp_record_path("--disable-calibrate 1", "T"), "--disable-calibrate 1");
  EXPECT_EQ(stamp_record_path("--record --foo", "T"), "--record --foo") << "no path: untouched";
  EXPECT_EQ(stamp_record_path("", "T"), "");
}

TEST(FrozenDriverArgs, RewritesOnlyWhenMarkerExists)
{
  using libsurvive_ros2::frozen_driver_args;
  const std::string session = "--force-calibrate 1 --configfile /c.json --light-relock-timeout 3";
  EXPECT_EQ(frozen_driver_args(session, false), session);
  EXPECT_EQ(
    frozen_driver_args(session, true),
    "--configfile /c.json --light-relock-timeout 3 --disable-calibrate 1");
  EXPECT_EQ(frozen_driver_args("--force-calibrate --configfile /c.json", true),
    "--configfile /c.json --disable-calibrate 1") << "flag without value";
  EXPECT_EQ(frozen_driver_args("--disable-calibrate 1 --configfile /c.json", true),
    "--disable-calibrate 1 --configfile /c.json") << "already frozen: unchanged";
  EXPECT_EQ(frozen_driver_args("", true), "--disable-calibrate 1");
}
