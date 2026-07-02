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
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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
// only the smoothed optical residual ("res_error_light_avg", i.e. the tracker's
// light_residuals_all), which is the value libsurvive itself thresholds against
// light-error-threshold to decide tracking is lost. The name check rejects every
// other series cheaply. Runs on the libsurvive worker thread.
static void datalog_func(
  SurviveObject * so, const char * name, const FLT * values, size_t length)
{
  if (_singleton == nullptr || so == nullptr || name == nullptr ||
    values == nullptr || length == 0)
  {
    return;
  }
  if (std::strcmp(name, "res_error_light_avg") != 0) {
    return;
  }
  _singleton->record_light_residual(so->serial_number, values[0]);
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

  // Setup driver parameters.
  std::string driver_args;
  this->declare_parameter("driver_args", "--force-recalibrate 1");
  this->get_parameter("driver_args", driver_args);
  std::vector<const char *> args;
  std::stringstream driver_ss(driver_args);
  std::string token;
  while (getline(driver_ss, token, ' ')) {
    args.emplace_back(token.c_str());
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

void Component::publish_diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->now();
  array.header.frame_id = tracking_frame_;

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
        const bool calibrated = bsd->PositionSet && bsd->OOTXSet;
        status.level = calibrated ?
          diagnostic_msgs::msg::DiagnosticStatus::OK :
          diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = calibrated ? "calibrated" : "calibrating";
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
      }
    } else {
      SurviveObject * so = survive_simple_get_survive_object(it);
      if (so == nullptr) {
        continue;  // external / unknown object — nothing to report
      }
      status.name = "libsurvive/tracker/" + serial;
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "tracking";

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

      add_kv(status, "pose_confidence", num(so->poseConfidence));
      add_kv(status, "light_residual", num(residual));
      add_kv(status, "pose_age_s", num(pose_age_s));
      add_kv(status, "charging", so->charging ? "true" : "false");
      add_kv(status, "charge_percent", std::to_string(static_cast<int>(so->charge)));
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
