#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>

#include <stepit/agent.h>
#include <stepit/policy_neuro/subscriber_action.h>
#include <stepit/policy_neuro_ros2/wbc_command_source.h>
#include <stepit/ros2/node.h>

namespace stepit::neuro_policy {
WbcCommandSource::WbcCommandSource(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "wbc_command")) {
  yml::Node subscriber_cfg = config_["wbc_command_subscriber"];
  auto [topic, topic_type, qos] = parseTopicInfo(subscriber_cfg, "/wbc_command", "std_msgs/msg/Float32MultiArray");
  STEPIT_ASSERT(topic_type == "std_msgs/msg/Float32MultiArray",
                "WbcCommandSource only supports std_msgs/msg/Float32MultiArray, but got '{}'.", topic_type);
  topic_ = topic;
  subscriber_cfg["neutral_height"].to(neutral_height_, true);
  subscriber_cfg["min_height"].to(min_height_, true);
  subscriber_cfg["max_height"].to(max_height_, true);
  subscriber_cfg["timeout_threshold"].to(timeout_threshold_, true);
  subscriber_cfg["default_enabled"].to(default_command_enabled_, true);
  STEPIT_ASSERT(min_height_ <= neutral_height_ and neutral_height_ <= max_height_,
                "neutral_height ({}) must be within [{}, {}].", neutral_height_, min_height_, max_height_);
  STEPIT_ASSERT(max_height_ > min_height_, "max_height ({}) must be greater than min_height ({}).", max_height_,
                min_height_);
  STEPIT_ASSERT(timeout_threshold_ > 0.0F, "timeout_threshold must be positive.");

  std::vector<float> default_left_pose;
  std::vector<float> default_right_pose;
  subscriber_cfg["default_left_pose"].to(default_left_pose);
  subscriber_cfg["default_right_pose"].to(default_right_pose);
  STEPIT_ASSERT(default_left_pose.size() == kPoseSize and default_right_pose.size() == kPoseSize,
                "default_left_pose and default_right_pose must each have {} values.", kPoseSize);
  STEPIT_ASSERT(isValidPose(default_left_pose, 0) and isValidPose(default_right_pose, 0),
                "Default WBC hand poses must contain finite position, non-zero tangent, and non-zero normal vectors.");
  for (std::size_t i{}; i < kPoseSize; ++i) {
    left_hold_[static_cast<Eigen::Index>(i)]  = default_left_pose[i];
    right_hold_[static_cast<Eigen::Index>(i)] = default_right_pose[i];
  }
  default_command_[3] = neutral_height_;
  default_command_.segment(static_cast<Eigen::Index>(kLeftOffset), static_cast<Eigen::Index>(kPoseSize)) = left_hold_;
  default_command_.segment(static_cast<Eigen::Index>(kRightOffset), static_cast<Eigen::Index>(kPoseSize)) = right_hold_;
  external_command_ = default_command_;
  command_ = default_command_;

  wbc_command_id_ = registerProvision("wbc_command", kCommandSize);
  command_sub_    = getNode()->create_subscription<std_msgs::msg::Float32MultiArray>(
      topic_, qos, std::bind(&WbcCommandSource::callback, this, std::placeholders::_1));

  yml::Node teleop_cfg = config_["teleop"];
  teleop_cfg["enabled"].to(teleop_enabled_, true);
  if (teleop_enabled_) {
    teleop_cfg["joystick_deadzone"].to(joystick_deadzone_, true);
    teleop_cfg["timeout_threshold"].to(teleop_timeout_, true);
    teleop_cfg["height_increment"].to(height_increment_, true);
    teleop_cfg["position_rate"].to(position_rate_, true);
    teleop_cfg["rotation_rate"].to(rotation_rate_, true);
    std::array<float, 2> velocity_x_range;
    std::array<float, 2> velocity_y_range;
    std::array<float, 2> yaw_rate_range;
    teleop_cfg["velocity_x_range"].to(velocity_x_range, true);
    teleop_cfg["velocity_y_range"].to(velocity_y_range, true);
    teleop_cfg["yaw_rate_range"].to(yaw_rate_range, true);
    vx_min_ = velocity_x_range[0];
    vx_max_ = velocity_x_range[1];
    vy_min_ = velocity_y_range[0];
    vy_max_ = velocity_y_range[1];
    wz_min_ = yaw_rate_range[0];
    wz_max_ = yaw_rate_range[1];

    yml::Node scaling_cfg = teleop_cfg["height_velocity_scaling"];
    scaling_cfg["enabled"].to(height_velocity_scaling_enabled_, true);
    scaling_cfg["min_height"].to(height_velocity_min_, true);
    scaling_cfg["full_speed_height"].to(height_velocity_full_speed_, true);
    scaling_cfg["xy_min_scale"].to(height_velocity_xy_min_scale_, true);
    scaling_cfg["yaw_min_scale"].to(height_velocity_yaw_min_scale_, true);
    scaling_cfg["power"].to(height_velocity_scale_power_, true);

    STEPIT_ASSERT(joystick_deadzone_ >= 0.0F and joystick_deadzone_ < 1.0F,
                  "teleop.joystick_deadzone must be in [0, 1).");
    STEPIT_ASSERT(teleop_timeout_ > 0.0F and height_increment_ > 0.0F and position_rate_ >= 0.0F and
                      rotation_rate_ >= 0.0F,
                  "teleop timeout, height increment, position rate, and rotation rate must be positive.");
    STEPIT_ASSERT(vx_min_ < 0.0F and vx_max_ > 0.0F and vy_min_ < 0.0F and vy_max_ > 0.0F and wz_min_ < 0.0F and
                      wz_max_ > 0.0F,
                  "teleop velocity ranges must straddle zero.");
    STEPIT_ASSERT(height_velocity_min_ < height_velocity_full_speed_,
                  "teleop height_velocity_scaling full_speed_height must exceed min_height.");

    auto [packet_topic, packet_type, packet_qos] =
        parseTopicInfo(teleop_cfg["packet_publisher"], "teleop_command", "std_msgs/msg/Float32MultiArray");
    auto [wbc_topic, wbc_type, wbc_qos] =
        parseTopicInfo(teleop_cfg["wbc_publisher"], topic_, "std_msgs/msg/Float32MultiArray");
    STEPIT_ASSERT(packet_type == "std_msgs/msg/Float32MultiArray" and wbc_type == "std_msgs/msg/Float32MultiArray",
                  "teleop packet and WBC publishers must use std_msgs/msg/Float32MultiArray.");
    teleop_packet_pub_ = getNode()->create_publisher<std_msgs::msg::Float32MultiArray>(packet_topic, packet_qos);
    if (wbc_topic == topic_) {
      STEPIT_WARN(
          "teleop.wbc_publisher topic '{}' matches the external WBC subscriber; disabling the preview publisher to "
          "prevent command feedback. Use a separate topic such as 'teleop_wbc_command'.",
          wbc_topic);
    } else {
      teleop_wbc_pub_ = getNode()->create_publisher<std_msgs::msg::Float32MultiArray>(wbc_topic, wbc_qos);
    }
    timestep_          = 1.0F / static_cast<float>(policy_spec.control_freq);
  }
}

bool WbcCommandSource::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  reported_timeout_ = false;
  command_enabled_.store(default_command_enabled_, std::memory_order_release);
  external_command_ = default_command_;
  command_          = default_command_;
  received_         = false;
  joystick_rules_.clear();
  if (teleop_enabled_) {
    resetTeleop();
    joystick_rules_.emplace_back([this](const joystick::State &js) -> std::string {
      // Preserve StepIt's trigger-modified Agent controls (stand/policy/etc.).
      if (js.lt() > 0.9F) return "";
      return fmt::format("Policy/WbcTeleop/Input:{},{},{},{},{},{},{},{},{},{}", js.las_x(), js.las_y(), js.ras_x(),
                         js.ras_y(), static_cast<int>(js.Left().pressed), static_cast<int>(js.Right().pressed),
                         static_cast<int>(js.Up().on_press), static_cast<int>(js.Down().on_press),
                         static_cast<int>(js.A().on_press), static_cast<int>(js.X().on_press));
    });
  }
  // L1+Y switches between joystick teleop and the external 22D ROS2 command.
  joystick_rules_.emplace_back([](const joystick::State &js) -> std::string {
    return js.LB().pressed and js.Y().on_press ? "Policy/WbcCommand/SwitchSubscriber" : "";
  });
  STEPIT_INFO("WBC control starts in {} mode; LB+Y toggles joystick and external ROS2 commands.",
              teleop_enabled_ and not default_command_enabled_ ? "joystick teleop" : "external ROS2");
  return true;
}

bool WbcCommandSource::update(const LowState &, ControlRequests &requests, FieldMap &context) {
  if (teleop_enabled_) {
    for (auto &&request : requests.filterByChannel("Policy/WbcTeleop")) {
      handleTeleopRequest(std::move(request));
    }
  }
  for (auto &&request : requests.filterByChannel("Policy/WbcCommand")) {
    handleCommandRequest(std::move(request));
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const bool external_enabled = command_enabled_.load(std::memory_order_acquire);
  if (teleop_enabled_ and not external_enabled) {
    updateTeleopCommand();
    publishTeleopCommand();
    context[wbc_command_id_] = command_;
    return true;
  }
  if (not external_enabled) {
    // Receiving a message is intentionally independent from approval. This
    // keeps the latest valid command ready, but never exposes it to the actor
    // before the operator presses L1+Y.
    reported_timeout_ = false;
    context[wbc_command_id_] = default_command_;
    return true;
  }
  if (received_ and getElapsedTime(command_stamp_) > timeout_threshold_) {
    if (not reported_timeout_) {
      STEPIT_WARN("WBC command from '{}' timed out after {:.3f}s; using neutral height and valid default hand targets.",
                  topic_, getElapsedTime(command_stamp_));
      reported_timeout_ = true;
    }
    context[wbc_command_id_] = default_command_;
    return true;
  }
  context[wbc_command_id_] = external_command_;
  return true;
}

void WbcCommandSource::exit() {
  command_enabled_.store(false, std::memory_order_release);
  joystick_rules_.clear();
}

void WbcCommandSource::callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (not applyCommand(msg->data)) {
    STEPIT_WARN("Rejected WBC command from '{}': expected 22 values with finite base command and valid hand poses.", topic_);
    return;
  }
  command_stamp_    = getNode()->now();
  received_         = true;
  reported_timeout_ = false;
}

void WbcCommandSource::handleCommandRequest(ControlRequest request) {
  switch (lookupAction(request.action(), kSubscriberActionMap)) {
    case SubscriberAction::kEnableSubscriber:
      command_enabled_.store(true, std::memory_order_release);
      request.response(kSuccess);
      STEPIT_LOG(kStartSubscribingTemplate, "external WBC command");
      break;
    case SubscriberAction::kDisableSubscriber:
      command_enabled_.store(false, std::memory_order_release);
      request.response(kSuccess);
      STEPIT_LOG(kStopSubscribingTemplate, "external WBC command");
      break;
    case SubscriberAction::kSwitchSubscriber: {
      const bool enabled = not command_enabled_.load(std::memory_order_relaxed);
      command_enabled_.store(enabled, std::memory_order_release);
      request.response(kSuccess);
      STEPIT_LOG(enabled ? kStartSubscribingTemplate : kStopSubscribingTemplate, "external WBC command");
      break;
    }
    default:
      request.response(kUnrecognizedRequest);
      break;
  }
}

bool WbcCommandSource::applyCommand(const std::vector<float> &data) {
  if (data.size() != kCommandSize) return false;
  for (std::size_t i{}; i < kLeftOffset; ++i) {
    if (not std::isfinite(data[i])) return false;
  }

  const bool left_missing  = isMissingPose(data, kLeftOffset);
  const bool right_missing = isMissingPose(data, kRightOffset);
  if ((not left_missing and not isValidPose(data, kLeftOffset)) or
      (not right_missing and not isValidPose(data, kRightOffset))) {
    return false;
  }

  external_command_.head<3>() = Eigen::Map<const Arr3f>(data.data());
  external_command_[3]        = std::clamp(data[3], min_height_, max_height_);
  if (not left_missing) {
    for (std::size_t i{}; i < kPoseSize; ++i) left_hold_[static_cast<Eigen::Index>(i)] = data[kLeftOffset + i];
  }
  if (not right_missing) {
    for (std::size_t i{}; i < kPoseSize; ++i) right_hold_[static_cast<Eigen::Index>(i)] = data[kRightOffset + i];
  }
  external_command_.segment(static_cast<Eigen::Index>(kLeftOffset), static_cast<Eigen::Index>(kPoseSize)) = left_hold_;
  external_command_.segment(static_cast<Eigen::Index>(kRightOffset), static_cast<Eigen::Index>(kPoseSize)) = right_hold_;
  return true;
}

void WbcCommandSource::handleTeleopRequest(ControlRequest request) {
  float las_x, las_y, ras_x, ras_y;
  float dpad_left, dpad_right, dpad_up, dpad_down, a_pressed, x_pressed;
  if (not request.parseArgument("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", las_x, las_y, ras_x, ras_y, dpad_left,
                                dpad_right, dpad_up, dpad_down, a_pressed, x_pressed) or
      not(std::isfinite(las_x) and std::isfinite(las_y) and std::isfinite(ras_x) and std::isfinite(ras_y))) {
    request.response(kIncorrectArgument);
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (a_pressed > 0.5F) {
    arm_mode_ = not arm_mode_;
    STEPIT_INFO("WBC teleop mode: {} (active hand: {}).", arm_mode_ ? "arm" : "locomotion",
                active_right_hand_ ? "right" : "left");
  }
  if (x_pressed > 0.5F) {
    active_right_hand_ = not active_right_hand_;
    STEPIT_INFO("WBC teleop active hand: {} (mode: {}).", active_right_hand_ ? "right" : "left",
                arm_mode_ ? "arm" : "locomotion");
  }

  const float lx = applyDeadzone(las_x, joystick_deadzone_);
  const float ly = applyDeadzone(las_y, joystick_deadzone_);
  const float rx = applyDeadzone(ras_x, joystick_deadzone_);
  const float ry = applyDeadzone(ras_y, joystick_deadzone_);
  base_command_[0] = scaleAxis(-ly, vx_min_, vx_max_);
  base_command_[1] = scaleAxis(-lx, vy_min_, vy_max_);
  base_command_[2] = scaleAxis(-rx, wz_min_, wz_max_);
  arm_axes_[0]     = -ly;
  arm_axes_[1]     = -lx;
  arm_axes_[2]     = -ry;
  arm_axes_[3]     = static_cast<float>(dpad_right > 0.5F) - static_cast<float>(dpad_left > 0.5F);
  arm_axes_[4]     = static_cast<float>(dpad_up > 0.5F) - static_cast<float>(dpad_down > 0.5F);
  arm_axes_[5]     = -rx;

  if (not arm_mode_) {
    if (dpad_up > 0.5F) base_command_[3] += height_increment_;
    if (dpad_down > 0.5F) base_command_[3] -= height_increment_;
    base_command_[3] = std::clamp(base_command_[3], min_height_, max_height_);
  }
  teleop_stamp_ = getNode()->now();
  request.response(kSuccess);
}

void WbcCommandSource::resetTeleop() {
  arm_mode_          = false;
  active_right_hand_ = true;
  base_command_.setZero();
  base_command_[3] = neutral_height_;
  arm_axes_.setZero();
  teleop_left_target_ = default_command_.segment(static_cast<Eigen::Index>(kLeftOffset), static_cast<Eigen::Index>(kPoseSize));
  teleop_right_target_ = default_command_.segment(static_cast<Eigen::Index>(kRightOffset), static_cast<Eigen::Index>(kPoseSize));
  teleop_stamp_ = getNode()->now();
}

void WbcCommandSource::updateTeleopCommand() {
  if (getElapsedTime(teleop_stamp_) > teleop_timeout_) {
    base_command_.head<3>().setZero();
    arm_axes_.setZero();
  }

  ArrXf packet = ArrXf::Zero(12);
  packet[0] = arm_mode_ ? 1.0F : 0.0F;
  packet[1] = active_right_hand_ ? 1.0F : 0.0F;
  packet.segment<4>(2) = base_command_;
  if (arm_mode_) {
    packet.head<3>().setZero();
    packet.segment<6>(6) = arm_axes_;
    ArrXf &target = active_right_hand_ ? teleop_right_target_ : teleop_left_target_;
    const Arr3f position_axis = arm_axes_.head<3>();
    const Arr3f rotation_axis = arm_axes_.tail<3>();
    integratePose(target, position_axis, rotation_axis, position_rate_, rotation_rate_, timestep_);
  }

  command_ = default_command_;
  command_.head<3>() = packet.segment<3>(2);
  command_[3]        = std::clamp(packet[5], min_height_, max_height_);
  if (height_velocity_scaling_enabled_) {
    float ratio = (command_[3] - height_velocity_min_) / (height_velocity_full_speed_ - height_velocity_min_);
    ratio       = std::clamp(ratio, 0.0F, 1.0F);
    const float shaped = std::pow(ratio, height_velocity_scale_power_);
    const float xy_scale = height_velocity_xy_min_scale_ + (1.0F - height_velocity_xy_min_scale_) * shaped;
    const float yaw_scale = height_velocity_yaw_min_scale_ + (1.0F - height_velocity_yaw_min_scale_) * shaped;
    command_[0] *= xy_scale;
    command_[1] *= xy_scale;
    command_[2] *= yaw_scale;
  }
  command_.segment(static_cast<Eigen::Index>(kLeftOffset), static_cast<Eigen::Index>(kPoseSize)) = teleop_left_target_;
  command_.segment(static_cast<Eigen::Index>(kRightOffset), static_cast<Eigen::Index>(kPoseSize)) = teleop_right_target_;
}

void WbcCommandSource::publishTeleopCommand() {
  std_msgs::msg::Float32MultiArray packet;
  packet.data.resize(12);
  packet.data[0] = arm_mode_ ? 1.0F : 0.0F;
  packet.data[1] = active_right_hand_ ? 1.0F : 0.0F;
  packet.data[2] = arm_mode_ ? 0.0F : base_command_[0];
  packet.data[3] = arm_mode_ ? 0.0F : base_command_[1];
  packet.data[4] = arm_mode_ ? 0.0F : base_command_[2];
  packet.data[5] = base_command_[3];
  for (Eigen::Index i{}; i < 6; ++i) packet.data[static_cast<std::size_t>(i + 6)] = arm_mode_ ? arm_axes_[i] : 0.0F;
  teleop_packet_pub_->publish(packet);

  std_msgs::msg::Float32MultiArray wbc;
  wbc.data.assign(command_.data(), command_.data() + command_.size());
  if (teleop_wbc_pub_) teleop_wbc_pub_->publish(wbc);
}

float WbcCommandSource::applyDeadzone(float value, float deadzone) {
  return std::abs(value) < deadzone ? 0.0F : std::clamp(value, -1.0F, 1.0F);
}

float WbcCommandSource::scaleAxis(float value, float lower, float upper) {
  return value >= 0.0F ? value * std::abs(upper) : value * std::abs(lower);
}

void WbcCommandSource::integratePose(ArrXf &pose, const Arr3f &position_axis, const Arr3f &rotation_axis,
                                     float position_rate, float rotation_rate, float timestep) {
  pose.head<3>() += position_axis * (position_rate * timestep);

  Eigen::Vector3f tangent = pose.segment<3>(3).matrix();
  Eigen::Vector3f normal  = pose.segment<3>(6).matrix();
  tangent.normalize();
  normal = (normal - tangent * tangent.dot(normal)).normalized();
  Eigen::Matrix3f rotation;
  rotation.col(0) = tangent;
  rotation.col(1) = normal.cross(tangent).normalized();
  rotation.col(2) = normal;

  const Eigen::Vector3f delta = rotation_axis.matrix() * (rotation_rate * timestep);
  const float angle = delta.norm();
  Eigen::Quaternionf target{rotation};
  if (angle > 1.0e-6F) target = Eigen::Quaternionf{Eigen::AngleAxisf{angle, delta / angle}} * target;
  const Eigen::Matrix3f updated = target.normalized().toRotationMatrix();
  pose.segment<3>(3) = updated.col(0).array();
  pose.segment<3>(6) = updated.col(2).array();
}

bool WbcCommandSource::isMissingPose(const std::vector<float> &data, std::size_t offset) {
  return std::all_of(data.begin() + static_cast<std::ptrdiff_t>(offset),
                     data.begin() + static_cast<std::ptrdiff_t>(offset + kPoseSize),
                     [](float value) { return std::isnan(value); });
}

bool WbcCommandSource::isValidPose(const std::vector<float> &data, std::size_t offset) {
  for (std::size_t i{}; i < kPoseSize; ++i) {
    if (not std::isfinite(data[offset + i])) return false;
  }
  const Eigen::Map<const Arr3f> tangent(data.data() + static_cast<std::ptrdiff_t>(offset + 3));
  const Eigen::Map<const Arr3f> normal(data.data() + static_cast<std::ptrdiff_t>(offset + 6));
  const float tangent_norm = tangent.matrix().norm();
  const float normal_norm  = normal.matrix().norm();
  if (tangent_norm < 1.0e-4F or normal_norm < 1.0e-4F) return false;
  return std::abs(tangent.matrix().dot(normal.matrix()) / (tangent_norm * normal_norm)) < 1.0e-3F;
}

STEPIT_REGISTER_MODULE(wbc_command_source, kDefPriority, Module::make<WbcCommandSource>);
STEPIT_REGISTER_FIELD_SOURCE(wbc_command, kDefPriority, Module::make<WbcCommandSource>);
}  // namespace stepit::neuro_policy
