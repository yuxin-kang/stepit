#ifndef STEPIT_NEURO_POLICY_ROS2_WBC_COMMAND_SOURCE_H_
#define STEPIT_NEURO_POLICY_ROS2_WBC_COMMAND_SOURCE_H_

#include <atomic>
#include <array>
#include <cstddef>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <stepit/joystick/joystick.h>
#include <stepit/policy_neuro/module.h>

namespace stepit::neuro_policy {
// Provides the trained 22D WBC command directly from one ROS2 message. Hand
// targets are pelvis/base-frame position + tangent + normal; a missing hand is
// represented by nine NaNs and retains the most recent valid target.
class WbcCommandSource : public Module {
 public:
  WbcCommandSource(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void exit() override;

 private:
  void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  bool applyCommand(const std::vector<float> &data);
  void handleCommandRequest(ControlRequest request);
  void handleTeleopRequest(ControlRequest request);
  void resetTeleop();
  void updateTeleopCommand();
  void publishTeleopCommand();
  static float applyDeadzone(float value, float deadzone);
  static float scaleAxis(float value, float lower, float upper);
  static void integratePose(ArrXf &pose, const Arr3f &position_axis, const Arr3f &rotation_axis, float position_rate,
                            float rotation_rate, float timestep);
  static bool isMissingPose(const std::vector<float> &data, std::size_t offset);
  static bool isValidPose(const std::vector<float> &data, std::size_t offset);

  static constexpr std::size_t kCommandSize = 22;
  static constexpr std::size_t kPoseSize    = 9;
  static constexpr std::size_t kLeftOffset  = 4;
  static constexpr std::size_t kRightOffset = 13;

  FieldId wbc_command_id_{};
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr teleop_packet_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr teleop_wbc_pub_;
  std::vector<JoystickRule> joystick_rules_;
  std::string topic_{"/wbc_command"};
  float neutral_height_{0.8F};
  float min_height_{0.3F};
  float max_height_{0.8F};
  float timeout_threshold_{0.25F};
  bool default_command_enabled_{false};
  // false: joystick teleop (when configured), true: external ROS2 WBC.
  std::atomic<bool> command_enabled_{false};

  // Training-side LocoManiRawJs protocol.  The raw 12D packet is
  // [arm_mode, active_hand, vx, vy, wz, height, dx, dy, dz, droll, dpitch, dyaw].
  bool teleop_enabled_{false};
  bool arm_mode_{false};
  bool active_right_hand_{true};
  float joystick_deadzone_{0.1F};
  float teleop_timeout_{0.25F};
  float height_increment_{0.02F};
  float position_rate_{0.25F};
  float rotation_rate_{1.0F};
  float vx_min_{-0.5F};
  float vx_max_{1.5F};
  float vy_min_{-0.5F};
  float vy_max_{0.5F};
  float wz_min_{-1.5F};
  float wz_max_{1.5F};
  bool height_velocity_scaling_enabled_{true};
  float height_velocity_min_{0.4F};
  float height_velocity_full_speed_{0.75F};
  float height_velocity_xy_min_scale_{0.2F};
  float height_velocity_yaw_min_scale_{0.5F};
  float height_velocity_scale_power_{2.0F};
  float timestep_{0.02F};
  ArrXf base_command_{ArrXf::Zero(4)};
  ArrXf arm_axes_{ArrXf::Zero(6)};
  ArrXf teleop_left_target_{ArrXf::Zero(kPoseSize)};
  ArrXf teleop_right_target_{ArrXf::Zero(kPoseSize)};
  rclcpp::Time teleop_stamp_{0, 0, RCL_ROS_TIME};

  std::mutex mutex_;
  ArrXf default_command_{ArrXf::Zero(kCommandSize)};
  ArrXf external_command_{ArrXf::Zero(kCommandSize)};
  ArrXf command_{ArrXf::Zero(kCommandSize)};
  ArrXf left_hold_{ArrXf::Zero(kPoseSize)};
  ArrXf right_hold_{ArrXf::Zero(kPoseSize)};
  rclcpp::Time command_stamp_{0, 0, RCL_ROS_TIME};
  bool received_{false};
  bool reported_timeout_{false};
};
}  // namespace stepit::neuro_policy

#endif  // STEPIT_NEURO_POLICY_ROS2_WBC_COMMAND_SOURCE_H_
