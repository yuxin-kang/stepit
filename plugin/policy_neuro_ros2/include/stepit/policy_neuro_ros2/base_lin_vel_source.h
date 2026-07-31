#ifndef STEPIT_NEURO_POLICY_ROS2_BASE_LIN_VEL_SOURCE_H_
#define STEPIT_NEURO_POLICY_ROS2_BASE_LIN_VEL_SOURCE_H_

#include <mutex>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <stepit/policy_neuro/module.h>

namespace stepit::neuro_policy {
// Supplies the teacher policy's base-frame linear velocity from ROS odometry.
// nav_msgs/Odometry specifies twist in child_frame_id, so "body" is the
// normal setting. "world" is available for publishers that explicitly put
// twist.linear in the odometry/world frame.
class BaseLinVelSource : public Module {
 public:
  BaseLinVelSource(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  FieldId base_lin_vel_id_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  std::string topic_{"/odometry"};
  std::string velocity_frame_{"body"};
  float timeout_threshold_{0.25F};

  std::mutex mutex_;
  Arr3f base_lin_vel_{Arr3f::Zero()};
  rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};
  bool received_{false};
  bool reported_timeout_{false};
};
}  // namespace stepit::neuro_policy

#endif  // STEPIT_NEURO_POLICY_ROS2_BASE_LIN_VEL_SOURCE_H_
