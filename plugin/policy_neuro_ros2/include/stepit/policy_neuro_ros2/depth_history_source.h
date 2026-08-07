#ifndef STEPIT_NEURO_POLICY_ROS2_DEPTH_HISTORY_SOURCE_H_
#define STEPIT_NEURO_POLICY_ROS2_DEPTH_HISTORY_SOURCE_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <stepit/policy_neuro/module.h>

namespace stepit::neuro_policy {
// Converts the continuous raw depth stream into the four-frame policy observation.
// Each received frame is cropped, clipped, and normalized before entering the
// source-owned history cache. The policy never receives raw camera frames.
class DepthHistorySource : public Module {
 public:
  DepthHistorySource(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  void arrayCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void acceptProcessedFrame(ArrXf processed);
  bool processImage(const sensor_msgs::msg::Image &image, ArrXf &processed) const;
  bool processFrame(const std::vector<float> &raw, ArrXf &processed) const;
  void applyGaussianBlur(ArrXf &frame) const;
  bool readyLocked() const;

  static constexpr std::size_t kDefaultHistoryFrames = 4;
  static constexpr std::size_t kDefaultPolicyHeight  = 18;
  static constexpr std::size_t kDefaultPolicyWidth   = 32;

  FieldId depth_history_id_{};
  FieldId depth_sequence_id_{};
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr depth_array_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  std::string topic_{"/depth_camera"};
  std::size_t raw_height_{36};
  std::size_t raw_width_{64};
  std::size_t crop_top_{18};
  std::size_t crop_bottom_{0};
  std::size_t crop_left_{16};
  std::size_t crop_right_{16};
  std::size_t history_frames_{kDefaultHistoryFrames};
  std::size_t policy_height_{kDefaultPolicyHeight};
  std::size_t policy_width_{kDefaultPolicyWidth};
  std::size_t policy_pixels_{kDefaultPolicyHeight * kDefaultPolicyWidth};
  float min_depth_{0.0F};
  float max_depth_{2.5F};
  std::size_t gaussian_blur_kernel_size_{1};
  float gaussian_blur_sigma_{1.0F};
  float timeout_threshold_{0.5F};

  mutable std::mutex mutex_;
  std::deque<ArrXf> history_;
  // Counts accepted camera frames.  The actor compares this value with its
  // recurrent state to decide whether to run the depth encoder again.
  std::uint64_t depth_sequence_id_value_{0};
  rclcpp::Time last_sample_stamp_{0, 0, RCL_ROS_TIME};
  std::size_t invalid_frame_count_{0};
  bool reported_not_ready_{false};
  bool reported_timeout_{false};
};
}  // namespace stepit::neuro_policy

#endif  // STEPIT_NEURO_POLICY_ROS2_DEPTH_HISTORY_SOURCE_H_
