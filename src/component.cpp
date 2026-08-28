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

// C++ system
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <format>  // NOLINT(build/include_order): cpplint predates C++20 <format>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>

// Other
#include "libsurvive_ros2/component.hpp"
#include "rclcpp_components/register_node_macro.hpp"


// Scale factor to move from G to m/s^2.
constexpr double SI_GRAVITY = 9.80665;

// We can only ever load one version of the driver, so we store a pointer to the instance of the
// driver here, so the IMU callback can push data to it.
libsurvive_ros2::Component * _singleton = nullptr;

static void imu_func(
  SurviveObject * so, int mask, const FLT * accelgyromag, uint32_t rawtime, int id)
{
  if (_singleton) {
    survive_default_imu_process(so, mask, accelgyromag, rawtime, id);
    FLT timecode = SurviveSensorActivations_runtime(
      &so->activations, so->activations.last_imu) / FLT(1e6);
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = std::string(so->serial_number) + "_imu";
    imu_msg.header.stamp = _singleton->get_ros_time("inertial", timecode);
    imu_msg.angular_velocity.x = accelgyromag[3];
    imu_msg.angular_velocity.y = accelgyromag[4];
    imu_msg.angular_velocity.z = accelgyromag[5];
    imu_msg.linear_acceleration.x = accelgyromag[0] * SI_GRAVITY;
    imu_msg.linear_acceleration.y = accelgyromag[1] * SI_GRAVITY;
    imu_msg.linear_acceleration.z = accelgyromag[2] * SI_GRAVITY;
    _singleton->publish_imu(imu_msg);
  }
}

// libsurvive emits many internal time series through the datalog hook; we keep
// two: the smoothed optical residual ("res_error_light_avg", i.e. the tracker's
// light_residuals_all, which libsurvive itself thresholds against
// light-error-threshold to decide tracking is lost) and the per-batch residual
// ("res_error_light_"), whose arrival is the only signal that light was actually
// folded into the pose filter. The name checks reject every other series
// cheaply. Runs on the libsurvive worker thread.
static void datalog_func(
  SurviveObject * so, const char * name, const FLT * values, size_t length)
{
  if (_singleton == nullptr || so == nullptr || name == nullptr ||
    values == nullptr || length == 0)
  {
    return;
  }
  if (std::strcmp(name, "res_error_light_avg") == 0) {
    _singleton->record_light_residual(so->serial_number, values[0]);
  } else if (std::strcmp(name, "res_error_light_") == 0) {
    _singleton->record_light_integrated(so->serial_number);
  }
}

// Light hooks: chain to libsurvive's own processing after noting the event.
// A sweep is a raw pulse attributed to a lighthouse channel; a sweep angle is
// that pulse turned into a usable measurement. Both run on libsurvive's threads.
static sweep_process_func _prev_sweep_fn = nullptr;
static sweep_angle_process_func _prev_sweep_angle_fn = nullptr;

static void sweep_func(
  SurviveObject * so, survive_channel channel, int sensor_id, survive_timecode timecode,
  bool flag)
{
  if (_singleton && so) {
    _singleton->record_sweep_hit(so->serial_number, survive_get_bsd_idx(so->ctx, channel));
  }
  if (_prev_sweep_fn) {
    _prev_sweep_fn(so, channel, sensor_id, timecode, flag);
  }
}

static void sweep_angle_func(
  SurviveObject * so, survive_channel channel, int sensor_id, survive_timecode timecode,
  int8_t plane, FLT angle)
{
  if (_singleton && so) {
    _singleton->record_solved_sweep(so->serial_number, survive_get_bsd_idx(so->ctx, channel));
  }
  if (_prev_sweep_angle_fn) {
    _prev_sweep_angle_fn(so, channel, sensor_id, timecode, plane, angle);
  }
}

static void ros_from_pose(
  geometry_msgs::msg::Transform * const tx, const SurvivePose & pose)
{
  tx->translation.x = pose.Pos[0];
  tx->translation.y = pose.Pos[1];
  tx->translation.z = pose.Pos[2];
  tx->rotation.w = pose.Rot[0];
  tx->rotation.x = pose.Rot[1];
  tx->rotation.y = pose.Rot[2];
  tx->rotation.z = pose.Rot[3];
}

namespace libsurvive_ros2
{

Component::Component(const rclcpp::NodeOptions & options)
: Node("libsurvive_ros2", options),
  actx_(nullptr),
  tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)),
  tf_static_broadcaster_(std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this))
{
  // Store the instance globally to be used by a C callback.
  _singleton = this;

  // Global parameters
  this->declare_parameter("tracking_frame", "libsurvive_frame");
  this->get_parameter("tracking_frame", tracking_frame_);
  this->declare_parameter("lighthouse_rate", 4.0);
  this->get_parameter("lighthouse_rate", lighthouse_rate_);

  // Setup topic for IMU.
  std::string imu_topic;
  this->declare_parameter("imu_topic", "imu");
  this->get_parameter("imu_topic", imu_topic);
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);

  // Setup topic for joystick.
  std::string joy_topic;
  this->declare_parameter("joy_topic", "joy");
  this->get_parameter("joy_topic", joy_topic);
  joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic, 10);

  // Setup topic for configuration.
  std::string cfg_topic;
  this->declare_parameter("cfg_topic", "cfg");
  this->get_parameter("cfg_topic", cfg_topic);
  cfg_publisher_ = this->create_publisher<diagnostic_msgs::msg::KeyValue>(cfg_topic, 10);

  // Diagnostics: per-lighthouse calibration flags + per-tracker pose confidence
  // and optical residual, for downstream calibration gating. Published from the
  // work loop at diagnostics_rate Hz.
  std::string diagnostics_topic;
  this->declare_parameter("publish_diagnostics", true);
  this->get_parameter("publish_diagnostics", publish_diagnostics_);
  this->declare_parameter("capture_light_residual", true);
  this->get_parameter("capture_light_residual", capture_light_residual_);
  this->declare_parameter("diagnostics_rate", 10.0);
  this->get_parameter("diagnostics_rate", diagnostics_rate_);
  this->declare_parameter("diagnostics_topic", "diagnostics");
  this->get_parameter("diagnostics_topic", diagnostics_topic);
  diagnostics_publisher_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);
  std::string tracking_status_topic;
  this->declare_parameter("tracking_status_topic", "tracking_status");
  this->get_parameter("tracking_status_topic", tracking_status_topic);
  tracking_status_publisher_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(tracking_status_topic, 10);

  // Tracking-health thresholds behind pose_source (see TrackingHealth).
  this->declare_parameter("imu_only_after_s", imu_only_after_s_);
  this->get_parameter("imu_only_after_s", imu_only_after_s_);
  this->declare_parameter("pose_stale_after_s", pose_stale_after_s_);
  this->get_parameter("pose_stale_after_s", pose_stale_after_s_);

  // Self-heal for a tracker that keeps streaming IMU but delivers no light after
  // total occlusion: libsurvive (our patched build) re-sends the lightcap mode
  // switch after this many seconds of darkness. <= 0 disables. The force
  // interval is a test knob that relocks periodically regardless of light.
  this->declare_parameter("light_relock_timeout_s", light_relock_timeout_s_);
  this->get_parameter("light_relock_timeout_s", light_relock_timeout_s_);
  this->declare_parameter("light_relock_force_interval_s", light_relock_force_interval_s_);
  this->get_parameter("light_relock_force_interval_s", light_relock_force_interval_s_);
  this->declare_parameter("wedge_restart_after_s", wedge_restart_after_s_);
  this->get_parameter("wedge_restart_after_s", wedge_restart_after_s_);
  this->declare_parameter("wedge_restart_min_relocks", wedge_restart_min_relocks_);
  this->get_parameter("wedge_restart_min_relocks", wedge_restart_min_relocks_);

  // Setup driver parameters.
  std::string driver_args;
  this->declare_parameter("driver_args", "--force-calibrate 1");
  this->get_parameter("driver_args", driver_args);
  // Parameters win over nothing: an explicit flag in driver_args is left alone.
  if (driver_args.find("--light-relock-timeout") == std::string::npos) {
    driver_args += std::format(" --light-relock-timeout {}", light_relock_timeout_s_);
  }
  if (light_relock_force_interval_s_ > 0.0 &&
    driver_args.find("--light-relock-force-interval") == std::string::npos)
  {
    driver_args +=
      std::format(" --light-relock-force-interval {}", light_relock_force_interval_s_);
  }
  // A respawned process would truncate the previous run's --record file — the
  // one artefact that explains why it respawned — so stamp the path per start.
  {
    const auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", std::localtime(&t));
    driver_args = stamp_record_path(driver_args, stamp);
  }
  RCLCPP_INFO(this->get_logger(), "libsurvive driver args: %s", driver_args.c_str());
  // libsurvive's parser treats argv[0] as the program name and scans from
  // argv[1], so prepend a placeholder or the first flag is dropped.
  std::vector<std::string> tokens{"libsurvive_ros2_node"};
  std::stringstream driver_ss(driver_args);
  std::string token;
  while (getline(driver_ss, token, ' ')) {
    if (!token.empty()) {
      tokens.push_back(token);
    }
  }
  // Build argv only after tokens is fully populated: each c_str() must point at
  // an owned string that outlives survive_simple_init (the old code took c_str()
  // of a single reused local, leaving dangling pointers).
  std::vector<const char *> args;
  args.reserve(tokens.size());
  for (const auto & t : tokens) {
    args.push_back(t.c_str());
  }

  // Try and initialize survive with the arguments supplied.
  actx_ = survive_simple_init(args.size(), const_cast<char **>(args.data()));
  if (actx_ == nullptr) {
    RCLCPP_FATAL(this->get_logger(), "Could not initialize the libsurvive context");
    return;
  }

  // Setup callback for reading IMU data.
  SurviveContext * ctx = survive_simple_get_ctx(actx_);
  survive_install_imu_fn(ctx, imu_func);

  // Capture the smoothed optical residual via the datalog hook; it is exposed on
  // no other interface. Installing the hook makes every SV_DATA_LOG site fire the
  // callback, so the callback rejects non-matching series cheaply (see datalog_func).
  if (capture_light_residual_) {
    survive_install_datalog_fn(ctx, datalog_func);
  }
  _prev_sweep_fn = survive_install_sweep_fn(ctx, sweep_func);
  _prev_sweep_angle_fn = survive_install_sweep_angle_fn(ctx, sweep_angle_func);

  // Initialize the survive thread.
  survive_simple_start_thread(actx_);

  // Start the work thread
  worker_thread_ = std::thread(&Component::work, this);
}

Component::~Component()
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up.");
  worker_thread_.join();

  RCLCPP_INFO(this->get_logger(), "Shutting down libsurvive driver");
  if (actx_) {
    survive_simple_close(actx_);
  }

  RCLCPP_INFO(this->get_logger(), "Clearing singleton instance");
  _singleton = nullptr;
}

rclcpp::Time Component::get_ros_time(const std::string & /*str*/, FLT timecode)
{
  return rclcpp::Time() + rclcpp::Duration(std::chrono::duration<double>(timecode));
}

void Component::record_light_residual(const std::string & serial, double value)
{
  std::lock_guard<std::mutex> lock(quality_mutex_);
  light_residuals_[serial] = value;
}

double Component::mono_now()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Caller holds health_mutex_.
TrackingHealth & Component::health_for(const std::string & serial)
{
  auto found = health_.find(serial);
  if (found == health_.end()) {
    found = health_.emplace(
      serial, TrackingHealth(0.5, imu_only_after_s_, pose_stale_after_s_)).first;
  }
  return found->second;
}

void Component::record_sweep_hit(const std::string & serial, int lh)
{
  std::lock_guard<std::mutex> lock(health_mutex_);
  health_for(serial).on_hit(lh, mono_now());
}

void Component::record_solved_sweep(const std::string & serial, int lh)
{
  std::lock_guard<std::mutex> lock(health_mutex_);
  health_for(serial).on_solved_sweep(lh, mono_now());
}

void Component::record_light_integrated(const std::string & serial)
{
  std::lock_guard<std::mutex> lock(health_mutex_);
  health_for(serial).on_light_integrated(mono_now());
}

void Component::publish_imu(const sensor_msgs::msg::Imu & msg)
{
  if (imu_publisher_) {
    imu_publisher_->publish(msg);
  }
}

namespace
{
void add_kv(
  diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  status.values.push_back(kv);
}

std::string num(double value)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.6g", value);
  return std::string(buf);
}
}  // namespace

void Component::maybe_exit_for_wedge(const std::string & serial, double light_age_s, int relocks)
{
  if (wedge_exit_requested_ ||
    !wedge_restart_due(light_age_s, relocks, wedge_restart_after_s_, wedge_restart_min_relocks_))
  {
    return;
  }
  wedge_exit_requested_ = true;
  RCLCPP_FATAL(
    this->get_logger(),
    "%s: IMU streaming but no light for %.1f s despite %d lightcap relocks — the "
    "tracker's light path is wedged (USB/firmware); exiting so the launch file "
    "respawns a fresh device open. Cover the tracker for a real occlusion test and "
    "this is expected; otherwise capture --record for offline replay.",
    serial.c_str(), light_age_s, relocks);
  // Flush every stdio stream (ROS console output, libsurvive's --record file)
  // and leave immediately: tearing rclcpp/libsurvive down from the worker
  // thread is not worth the risk, the point is a clean re-enumeration.
  std::fflush(nullptr);
  std::_Exit(kWedgeExitCode);
}

void Component::publish_diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->now();
  array.header.frame_id = tracking_frame_;

  SurviveContext * ctx = survive_simple_get_ctx(actx_);

  // Snapshot every tracker's light health once, under the lock, then build the
  // message lock-free. Lighthouse rows sum the per-tracker counts.
  std::map<std::string, TrackingHealth::Snapshot> health;
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    const double now = mono_now();
    for (auto & [serial, h] : health_) {
      health[serial] = h.snapshot(now);
    }
  }
  std::array<int, TrackingHealth::kMaxLighthouses> lh_hits{};
  std::array<int, TrackingHealth::kMaxLighthouses> lh_solved{};
  for (const auto & [serial, snap] : health) {
    for (std::size_t lh = 0; lh < TrackingHealth::kMaxLighthouses; ++lh) {
      lh_hits[lh] += snap.hits[lh];
      lh_solved[lh] += snap.solved[lh];
    }
  }

  // Lighthouse slot index -> serial, for the per-lighthouse breakdown on tracker rows.
  std::map<int, std::string> lh_serials;
  for (const SurviveSimpleObject * it = survive_simple_get_first_object(actx_); it != nullptr;
    it = survive_simple_get_next_object(actx_, it))
  {
    if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
      const BaseStationData * bsd = survive_simple_get_bsd(it);
      const char * serial_c = survive_simple_serial_number(it);
      if (bsd != nullptr && serial_c != nullptr) {
        lh_serials[static_cast<int>(bsd - ctx->bsd)] = serial_c;
      }
    }
  }

  for (const SurviveSimpleObject * it = survive_simple_get_first_object(actx_); it != nullptr;
    it = survive_simple_get_next_object(actx_, it))
  {
    const char * serial_c = survive_simple_serial_number(it);
    const std::string serial = serial_c ? serial_c : "";

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.hardware_id = serial;

    if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
      status.name = "libsurvive/lighthouse/" + serial;
      // bsd->* (and so->* below) are read directly from libsurvive's internal
      // state while its worker thread may be writing them — non-atomic reads that
      // can tear on the FLT arrays. Accepted for diagnostics: the values are only
      // ever advisory here, never used for control, and a torn sample self-corrects
      // on the next publish.
      BaseStationData * bsd = survive_simple_get_bsd(it);
      if (bsd == nullptr) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "no base station data";
      } else {
        const int idx = static_cast<int>(bsd - ctx->bsd);
        const bool calibrated = bsd->PositionSet && bsd->OOTXSet;
        const bool in_window =
          idx >= 0 && static_cast<std::size_t>(idx) < TrackingHealth::kMaxLighthouses;
        const int hits = in_window ? lh_hits[idx] : 0;
        const int solved = in_window ? lh_solved[idx] : 0;
        status.level = calibrated ?
          diagnostic_msgs::msg::DiagnosticStatus::OK :
          diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = calibrated ? "calibrated" : "calibrating";
        if (bsd->ootx_conflict_count > 0) {
          // A foreign base station shares this channel (see patches/): the stored
          // calibration was protected, but every sweep on the channel is now
          // ambiguous, so tracking through this lighthouse is unreliable.
          status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          status.message = std::format(
            "channel {} conflict: foreign base station LHB-{:08X} seen {} times; "
            "stored calibration kept (record mode)",
            static_cast<int>(bsd->mode), bsd->ootx_conflict_id, bsd->ootx_conflict_count);
        }
        add_kv(status, "position_set", bsd->PositionSet ? "true" : "false");
        add_kv(status, "ootx_set", bsd->OOTXSet ? "true" : "false");
        add_kv(status, "ootx_checked", bsd->OOTXChecked ? "true" : "false");
        add_kv(status, "disabled", bsd->disable ? "true" : "false");
        add_kv(status, "base_station_id", std::to_string(bsd->BaseStationID));
        add_kv(status, "mode", std::to_string(static_cast<int>(bsd->mode)));
        add_kv(status, "confidence", num(bsd->confidence));
        // variance is a SurviveAxisAnglePose: Pos[3] followed by AxisAngleRot[3],
        // i.e. 6 contiguous FLTs (3 position + 3 axis-angle rotation variances),
        // matching the 6-DOF layout in config.json. Not a quaternion pose.
        const FLT * var = &bsd->variance.Pos[0];
        add_kv(
          status, "variance",
          num(var[0]) + " " + num(var[1]) + " " + num(var[2]) + " " +
          num(var[3]) + " " + num(var[4]) + " " + num(var[5]));
        add_kv(status, "hits_500ms", std::to_string(hits));
        add_kv(status, "solved_sweeps_500ms", std::to_string(solved));
        add_kv(status, "visible", solved > 0 ? "true" : "false");
        add_kv(
          status, "ootx_conflict_id",
          bsd->ootx_conflict_count > 0 ? std::format("LHB-{:08X}", bsd->ootx_conflict_id) : "");
        add_kv(status, "ootx_conflict_count", std::to_string(bsd->ootx_conflict_count));
      }
    } else {
      SurviveObject * so = survive_simple_get_survive_object(it);
      if (so == nullptr) {
        continue;  // external / unknown object — nothing to report
      }
      status.name = "libsurvive/tracker/" + serial;

      // Age since this tracker's pose was last broadcast, in ROS time (set in
      // the work loop on each PoseUpdateEvent). Infinity until the first pose.
      double pose_age_s = std::numeric_limits<double>::infinity();
      const auto seen = last_pose_time_.find(serial);
      if (seen != last_pose_time_.end()) {
        pose_age_s = (this->now() - seen->second).seconds();
      }

      // record_light_residual() keys by so->serial_number (datalog thread), and
      // survive_simple_serial_number() returns that same field for trackers, so
      // `serial` is the identical key — use it for a single, consistent source.
      double residual = std::nan("");
      {
        std::lock_guard<std::mutex> lock(quality_mutex_);
        const auto found = light_residuals_.find(serial);
        if (found != light_residuals_.end()) {
          residual = found->second;
        }
      }

      TrackingHealth::Snapshot snap;
      const auto hs = health.find(serial);
      if (hs != health.end()) {
        snap = hs->second;
      }
      if (snap.pose_source == "light") {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "tracking";
      } else if (snap.pose_source == "imu_only") {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = std::format(
          "no light for {:.1f} s: pose is IMU dead-reckoning", snap.light_integrated_age_s);
      } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = std::format("no pose for {:.1f} s", pose_age_s);
      }

      // Existing keys first, unchanged: downstream monitors parse them by name.
      add_kv(status, "pose_confidence", num(so->poseConfidence));
      add_kv(status, "light_residual", num(residual));
      add_kv(status, "pose_age_s", num(pose_age_s));
      add_kv(status, "charging", so->charging ? "true" : "false");
      add_kv(status, "charge_percent", std::to_string(static_cast<int>(so->charge)));

      add_kv(status, "light_age_s", num(snap.light_age_s));
      add_kv(status, "light_integrated_age_s", num(snap.light_integrated_age_s));
      add_kv(status, "lighthouses_visible", std::to_string(snap.lighthouses_visible));
      add_kv(status, "pose_source", snap.pose_source);
      add_kv(status, "light_relocks", std::to_string(so->stats.light_relocks));
      add_kv(status, "wedge_restart_after_s", num(wedge_restart_after_s_));
      maybe_exit_for_wedge(serial, snap.light_age_s, static_cast<int>(so->stats.light_relocks));
      std::string hits_s;
      std::string solved_s;
      for (const auto & [idx, lh_serial] : lh_serials) {
        if (idx < 0 || static_cast<std::size_t>(idx) >= TrackingHealth::kMaxLighthouses) {
          continue;
        }
        hits_s += std::format("{}{}={}", hits_s.empty() ? "" : " ", lh_serial, snap.hits[idx]);
        solved_s +=
          std::format("{}{}={}", solved_s.empty() ? "" : " ", lh_serial, snap.solved[idx]);
      }
      add_kv(status, "lh_hits_500ms", hits_s);
      add_kv(status, "lh_solved_500ms", solved_s);
      tracking_status_publisher_->publish(status);
    }

    array.status.push_back(status);
  }

  diagnostics_publisher_->publish(array);
}

void Component::work()
{
  RCLCPP_INFO(this->get_logger(), "Start listening for events..");

  // Poll for events.
  struct SurviveSimpleEvent event = {};
  while (survive_simple_wait_for_event(
      actx_,
      &event) != SurviveSimpleEventType_Shutdown && rclcpp::ok())
  {
    // Business logic depends on the event type
    switch (event.event_type) {
      // TYPE: Pose update (limit to non-lighthouses only)
      case SurviveSimpleEventType_PoseUpdateEvent: {
          const struct SurviveSimplePoseUpdatedEvent * pose_event =
            survive_simple_get_pose_updated_event(&event);
          if (survive_simple_object_get_type(pose_event->object) !=
            SurviveSimpleObject_LIGHTHOUSE)
          {
            SurvivePose pose = {};
            auto timecode = survive_simple_object_get_latest_pose(pose_event->object, &pose);
            if (timecode > 0) {
              geometry_msgs::msg::TransformStamped pose_msg;
              pose_msg.header.stamp = this->get_ros_time("tracker", timecode);
              pose_msg.header.frame_id = tracking_frame_;
              pose_msg.child_frame_id = survive_simple_serial_number(pose_event->object);
              ros_from_pose(&pose_msg.transform, pose);
              tf_broadcaster_->sendTransform(pose_msg);
              // Stamp pose freshness in ROS time at receive — robust to
              // libsurvive's internal clock bases. Read by publish_diagnostics
              // (same worker thread, so no lock needed).
              last_pose_time_[pose_msg.child_frame_id] = this->now();
              {
                std::lock_guard<std::mutex> lock(health_mutex_);
                health_for(pose_msg.child_frame_id).on_pose(mono_now());
              }
            }
          }
          break;
        }

      // TYPE: Button update
      case SurviveSimpleEventType_ButtonEvent: {
          const struct SurviveSimpleButtonEvent * button_event = survive_simple_get_button_event(
            &event);
          auto obj = button_event->object;
          sensor_msgs::msg::Joy joy_msg;
          joy_msg.header.frame_id = survive_simple_serial_number(button_event->object);
          joy_msg.header.stamp = this->get_ros_time("button", button_event->time);
          joy_msg.axes.resize(SURVIVE_MAX_AXIS_COUNT * 2);
          joy_msg.buttons.resize(SURVIVE_BUTTON_MAX * 2);
          int64_t mask = survive_simple_object_get_button_mask(obj);
          mask |= (survive_simple_object_get_touch_mask(obj) << SURVIVE_BUTTON_MAX);
          for (int i = 0; i < SURVIVE_MAX_AXIS_COUNT * 2; i++) {
            joy_msg.axes[i] =
              static_cast<float>(survive_simple_object_get_input_axis(obj, (enum SurviveAxis)i));
          }
          for (int i = 0; i < mask && i < static_cast<int>(joy_msg.buttons.size()); i++) {
            joy_msg.buttons[i] = (mask >> i) & 1;
          }
          joy_publisher_->publish(joy_msg);
          break;
        }

      // TYPE: Configuration update
      case SurviveSimpleEventType_ConfigEvent: {
          const struct SurviveSimpleConfigEvent * config_event = survive_simple_get_config_event(
            &event);
          diagnostic_msgs::msg::KeyValue cfg_msg;
          cfg_msg.key = survive_simple_serial_number(config_event->object);
          cfg_msg.value = config_event->cfg;
          cfg_publisher_->publish(cfg_msg);
          break;
        }

      // TYPE: Device add event
      case SurviveSimpleEventType_DeviceAdded: {
          const struct SurviveSimpleObjectEvent * object_event = survive_simple_get_object_event(
            &event);
          RCLCPP_INFO(
            this->get_logger(), "A new device %s was added at time %lf",
            survive_simple_serial_number(object_event->object),
            this->get_ros_time("connect", object_event->time).seconds()
          );
          break;
        }

      // TYPE: no-op
      case SurviveSimpleEventType_None: {
          break;
        }

      // We should never get here.
      default:
        RCLCPP_WARN(this->get_logger(), "Unknown event");
        break;
    }

    // Always update the base stations
    auto time_now = this->get_clock()->now();
    if (time_now.seconds() - last_base_station_update_.seconds() > lighthouse_rate_) {
      last_base_station_update_ = time_now;
      for (const SurviveSimpleObject * it = survive_simple_get_first_object(actx_); it != 0;
        it = survive_simple_get_next_object(actx_, it))
      {
        if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
          SurvivePose pose = {};
          auto timecode = survive_simple_object_get_latest_pose(it, &pose);
          if (timecode > 0) {
            geometry_msgs::msg::TransformStamped pose_msg;
            pose_msg.header.stamp = this->get_ros_time("lighthouse", timecode);
            pose_msg.header.frame_id = tracking_frame_;
            pose_msg.child_frame_id = survive_simple_serial_number(it);
            ros_from_pose(&pose_msg.transform, pose);
            tf_static_broadcaster_->sendTransform(pose_msg);
          }
        }
      }
    }

    // Publish diagnostics at a decimated rate. The loop is event-driven, so gate
    // on wall-clock seconds (compared as doubles, matching the base-station gate
    // above) rather than on rclcpp::Time subtraction, which would require matching
    // clock sources. survive_simple_wait_for_event() wakes at least every 100 ms
    // (its condvar timeout) even when no events arrive, so a stalled/lost tracker
    // still gets diagnostics published at up to ~10 Hz — reporting the degradation
    // (rising pose_age_s) rather than looking like a dead node. A diagnostics_rate
    // above ~10 Hz is therefore capped by that wake interval during event starvation.
    if (publish_diagnostics_ && diagnostics_rate_ > 0.0) {
      const double now_s = this->get_clock()->now().seconds();
      if (now_s - last_diag_update_s_ >= 1.0 / diagnostics_rate_) {
        last_diag_update_s_ = now_s;
        publish_diagnostics();
      }
    }
  }
}

}  // namespace libsurvive_ros2

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(libsurvive_ros2::Component)
