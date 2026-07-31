#include <cmath>
#include <functional>

#include <Eigen/Geometry>

#include <stepit/policy_neuro_ros2/base_lin_vel_source.h>
#include <stepit/ros2/node.h>

namespace stepit::neuro_policy {
BaseLinVelSource::BaseLinVelSource(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "base_lin_vel")) {
  yml::Node subscriber_cfg = config_["base_lin_vel_subscriber"];
  auto [topic, topic_type, qos] = parseTopicInfo(subscriber_cfg, "/odometry", "nav_msgs/msg/Odometry");
  STEPIT_ASSERT(topic_type == "nav_msgs/msg/Odometry",
                "BaseLinVelSource only supports nav_msgs/msg/Odometry, but got '{}'.", topic_type);
  topic_ = topic;
  subscriber_cfg["timeout_threshold"].to(timeout_threshold_, true);
  subscriber_cfg["velocity_frame"].to(velocity_frame_, true);
  STEPIT_ASSERT(timeout_threshold_ > 0.0F, "base_lin_vel timeout_threshold must be positive.");
  STEPIT_ASSERT(velocity_frame_ == "body" or velocity_frame_ == "world",
                "base_lin_vel velocity_frame must be 'body' or 'world', but got '{}'.", velocity_frame_);

  base_lin_vel_id_ = registerProvision("base_lin_vel", 3);
  odometry_sub_    = getNode()->create_subscription<nav_msgs::msg::Odometry>(
      topic_, qos, std::bind(&BaseLinVelSource::callback, this, std::placeholders::_1));
}

bool BaseLinVelSource::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  // Keep the most recent sample: subscriptions run before policy activation,
  // and a reset must not discard a valid odometry sample already in flight.
  reported_timeout_ = false;
  return true;
}

bool BaseLinVelSource::update(const LowState &, ControlRequests &, FieldMap &context) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (not received_) {
    STEPIT_WARN("Base linear velocity from '{}' is not available; start the odometry publisher before the teacher policy.",
                topic_);
    return false;
  }
  const double age = getElapsedTime(stamp_);
  if (age > timeout_threshold_) {
    if (not reported_timeout_) {
      STEPIT_WARN("Base linear velocity from '{}' timed out after {:.3f}s (threshold {:.3f}s).", topic_, age,
                  timeout_threshold_);
      reported_timeout_ = true;
    }
    return false;
  }
  context[base_lin_vel_id_] = base_lin_vel_;
  return true;
}

void BaseLinVelSource::callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto &linear = msg->twist.twist.linear;
  if (not(std::isfinite(linear.x) and std::isfinite(linear.y) and std::isfinite(linear.z))) {
    STEPIT_WARN("Rejected non-finite base linear velocity from '{}'.", topic_);
    return;
  }

  Arr3f velocity{static_cast<float>(linear.x), static_cast<float>(linear.y), static_cast<float>(linear.z)};
  if (velocity_frame_ == "world") {
    const auto &orientation = msg->pose.pose.orientation;
    if (not(std::isfinite(orientation.w) and std::isfinite(orientation.x) and std::isfinite(orientation.y) and
            std::isfinite(orientation.z))) {
      STEPIT_WARN("Rejected world-frame odometry from '{}': pose orientation is not finite.", topic_);
      return;
    }
    Eigen::Quaternionf world_from_base(static_cast<float>(orientation.w), static_cast<float>(orientation.x),
                                       static_cast<float>(orientation.y), static_cast<float>(orientation.z));
    if (world_from_base.squaredNorm() < 1e-8F) {
      STEPIT_WARN("Rejected world-frame odometry from '{}': pose orientation has zero norm.", topic_);
      return;
    }
    world_from_base.normalize();
    velocity = (world_from_base.conjugate() * velocity.matrix()).array();
  }

  std::lock_guard<std::mutex> lock(mutex_);
  base_lin_vel_   = velocity;
  stamp_          = getNode()->now();
  received_       = true;
  reported_timeout_ = false;
}

STEPIT_REGISTER_MODULE(base_lin_vel_source, kDefPriority, Module::make<BaseLinVelSource>);
STEPIT_REGISTER_FIELD_SOURCE(base_lin_vel, kDefPriority, Module::make<BaseLinVelSource>);
}  // namespace stepit::neuro_policy
