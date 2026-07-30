#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>

#include <stepit/policy_neuro_ros2/depth_history_source.h>
#include <stepit/ros2/node.h>

namespace stepit::neuro_policy {
DepthHistorySource::DepthHistorySource(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "depth_history")) {
  yml::Node subscriber_cfg = config_["depth_subscriber"];
  auto [topic, topic_type, qos] = parseTopicInfo(subscriber_cfg, "/depth_camera", "sensor_msgs/msg/Image");
  topic_ = topic;
  subscriber_cfg["raw_height"].to(raw_height_, true);
  subscriber_cfg["raw_width"].to(raw_width_, true);
  subscriber_cfg["crop_top"].to(crop_top_, true);
  subscriber_cfg["crop_bottom"].to(crop_bottom_, true);
  subscriber_cfg["crop_left"].to(crop_left_, true);
  subscriber_cfg["crop_right"].to(crop_right_, true);
  subscriber_cfg["min_depth"].to(min_depth_, true);
  subscriber_cfg["max_depth"].to(max_depth_, true);
  subscriber_cfg["gaussian_blur_kernel_size"].to(gaussian_blur_kernel_size_, true);
  subscriber_cfg["gaussian_blur_sigma"].to(gaussian_blur_sigma_, true);
  subscriber_cfg["timeout_threshold"].to(timeout_threshold_, true);

  STEPIT_ASSERT(raw_height_ > crop_top_ + crop_bottom_ and raw_width_ > crop_left_ + crop_right_,
                "Depth crop ({}, {}, {}, {}) is invalid for raw size {}x{}.", crop_top_, crop_bottom_, crop_left_,
                crop_right_, raw_height_, raw_width_);
  STEPIT_ASSERT(raw_height_ - crop_top_ - crop_bottom_ == kPolicyHeight and
                    raw_width_ - crop_left_ - crop_right_ == kPolicyWidth,
                "Depth crop must produce the trained {}x{} image, but raw {}x{} with crop ({}, {}, {}, {}) produces {}x{}.",
                kPolicyHeight, kPolicyWidth, raw_height_, raw_width_, crop_top_, crop_bottom_, crop_left_, crop_right_,
                raw_height_ - crop_top_ - crop_bottom_, raw_width_ - crop_left_ - crop_right_);
  STEPIT_ASSERT(max_depth_ > min_depth_, "max_depth ({}) must be greater than min_depth ({}).", max_depth_, min_depth_);
  STEPIT_ASSERT(gaussian_blur_kernel_size_ > 0 and gaussian_blur_kernel_size_ % 2 == 1,
                "gaussian_blur_kernel_size ({}) must be a positive odd number.", gaussian_blur_kernel_size_);
  STEPIT_ASSERT(gaussian_blur_kernel_size_ <= std::min(kPolicyHeight, kPolicyWidth),
                "gaussian_blur_kernel_size ({}) exceeds the policy image size {}x{}.", gaussian_blur_kernel_size_,
                kPolicyHeight, kPolicyWidth);
  STEPIT_ASSERT(gaussian_blur_sigma_ > 0.0F, "gaussian_blur_sigma must be positive.");
  STEPIT_ASSERT(timeout_threshold_ > 0.0F, "timeout_threshold must be positive.");

  depth_history_id_ = registerProvision("depth_history", kHistoryFrames * kPolicyPixels);
  if (topic_type == "sensor_msgs/msg/Image") {
    depth_image_sub_ = getNode()->create_subscription<sensor_msgs::msg::Image>(
        topic_, qos, std::bind(&DepthHistorySource::imageCallback, this, std::placeholders::_1));
  } else if (topic_type == "std_msgs/msg/Float32MultiArray") {
    depth_array_sub_ = getNode()->create_subscription<std_msgs::msg::Float32MultiArray>(
        topic_, qos, std::bind(&DepthHistorySource::arrayCallback, this, std::placeholders::_1));
  } else {
    STEPIT_THROW("DepthHistorySource topic '{}' must use sensor_msgs/msg/Image or std_msgs/msg/Float32MultiArray, got '{}'.",
                 topic_, topic_type);
  }
}

bool DepthHistorySource::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  reported_not_ready_ = false;
  reported_timeout_   = false;
  if (not readyLocked()) {
    STEPIT_WARN("Depth history is not ready: need {} processed frames from '{}', got {}.", kHistoryFrames, topic_,
                history_.size());
    return false;
  }
  if (getElapsedTime(last_sample_stamp_) > timeout_threshold_) {
    STEPIT_WARN("Depth history is stale at reset: latest sampled frame from '{}' is {:.3f}s old.", topic_,
                getElapsedTime(last_sample_stamp_));
    return false;
  }
  return true;
}

bool DepthHistorySource::update(const LowState &, ControlRequests &, FieldMap &context) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (not readyLocked()) {
    if (not reported_not_ready_) {
      STEPIT_WARN("Depth history is not ready: need {} processed frames from '{}', got {}.", kHistoryFrames, topic_,
                  history_.size());
      reported_not_ready_ = true;
    }
    return false;
  }
  const double age = getElapsedTime(last_sample_stamp_);
  if (age > timeout_threshold_) {
    if (not reported_timeout_) {
      STEPIT_WARN("Depth history from '{}' timed out after {:.3f}s (threshold {:.3f}s).", topic_, age,
                  timeout_threshold_);
      reported_timeout_ = true;
    }
    return false;
  }

  ArrXf depth_history{kHistoryFrames * kPolicyPixels};
  for (std::size_t i{}; i < kHistoryFrames; ++i) {
    depth_history.segment(static_cast<Eigen::Index>(i * kPolicyPixels), static_cast<Eigen::Index>(kPolicyPixels)) =
        history_[i];
  }
  context[depth_history_id_] = std::move(depth_history);
  reported_not_ready_        = false;
  reported_timeout_          = false;
  return true;
}

void DepthHistorySource::arrayCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  ArrXf processed;
  if (not processFrame(msg->data, processed)) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_frame_count_;
    if (invalid_frame_count_ == 1 or invalid_frame_count_ % 100 == 0) {
      STEPIT_WARN("Rejected depth frame from '{}': expected {} finite values ({}x{}), got {}.", topic_,
                  raw_height_ * raw_width_, raw_height_, raw_width_, msg->data.size());
    }
    return;
  }
  acceptProcessedFrame(std::move(processed));
}

void DepthHistorySource::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  ArrXf processed;
  if (not processImage(*msg, processed)) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_frame_count_;
    if (invalid_frame_count_ == 1 or invalid_frame_count_ % 100 == 0) {
      STEPIT_WARN("Rejected depth image from '{}': expected {}x{} 32FC1 with valid row stride, got {}x{} '{}' step {}.",
                  topic_, raw_width_, raw_height_, msg->width, msg->height, msg->encoding, msg->step);
    }
    return;
  }
  acceptProcessedFrame(std::move(processed));
}

void DepthHistorySource::acceptProcessedFrame(ArrXf processed) {
  const rclcpp::Time stamp = getNode()->now();
  std::lock_guard<std::mutex> lock(mutex_);
  history_.push_back(std::move(processed));
  if (history_.size() > kHistoryFrames) history_.pop_front();
  last_sample_stamp_ = stamp;
  reported_not_ready_ = false;
  reported_timeout_   = false;
}

bool DepthHistorySource::processImage(const sensor_msgs::msg::Image &image, ArrXf &processed) const {
  if (image.encoding != "32FC1" or image.width != raw_width_ or image.height != raw_height_ or
      image.step < raw_width_ * sizeof(float) or image.data.size() < image.step * raw_height_) {
    return false;
  }

  const std::uint16_t endian_probe{1};
  const bool host_is_big_endian = *reinterpret_cast<const std::uint8_t *>(&endian_probe) == 0;
  const bool swap_bytes         = (image.is_bigendian != 0) != host_is_big_endian;
  std::vector<float> raw(raw_height_ * raw_width_);
  for (std::size_t row{}; row < raw_height_; ++row) {
    const auto *row_data = image.data.data() + row * image.step;
    for (std::size_t col{}; col < raw_width_; ++col) {
      std::array<std::uint8_t, sizeof(float)> bytes{};
      std::memcpy(bytes.data(), row_data + col * sizeof(float), sizeof(float));
      if (swap_bytes) std::reverse(bytes.begin(), bytes.end());
      float value{};
      std::memcpy(&value, bytes.data(), sizeof(float));
      raw[row * raw_width_ + col] = std::isfinite(value) ? value : max_depth_;
    }
  }
  return processFrame(raw, processed);
}

bool DepthHistorySource::processFrame(const std::vector<float> &raw, ArrXf &processed) const {
  if (raw.size() != raw_height_ * raw_width_) return false;
  for (float value : raw) {
    if (not std::isfinite(value)) return false;
  }

  processed.resize(kPolicyPixels);
  std::size_t output_index{};
  for (std::size_t row = crop_top_; row < raw_height_ - crop_bottom_; ++row) {
    for (std::size_t col = crop_left_; col < raw_width_ - crop_right_; ++col) {
      processed[static_cast<Eigen::Index>(output_index++)] = raw[row * raw_width_ + col];
    }
  }

  applyGaussianBlur(processed);
  const float depth_range = max_depth_ - min_depth_;
  processed = processed.unaryExpr([this, depth_range](float value) {
    return (std::clamp(value, min_depth_, max_depth_) - min_depth_) / depth_range;
  });
  return true;
}

void DepthHistorySource::applyGaussianBlur(ArrXf &frame) const {
  if (gaussian_blur_kernel_size_ == 1) return;

  const int radius = static_cast<int>(gaussian_blur_kernel_size_ / 2);
  std::vector<float> kernel(gaussian_blur_kernel_size_);
  float kernel_sum{};
  for (int offset = -radius; offset <= radius; ++offset) {
    const float value = std::exp(-0.5F * static_cast<float>(offset * offset) /
                                 (gaussian_blur_sigma_ * gaussian_blur_sigma_));
    kernel[static_cast<std::size_t>(offset + radius)] = value;
    kernel_sum += value;
  }
  for (float &value : kernel) value /= kernel_sum;

  auto reflect = [](int index, int size) {
    while (index < 0 or index >= size) {
      if (index < 0) index = -index;
      if (index >= size) index = 2 * size - 2 - index;
    }
    return index;
  };

  ArrXf horizontal{kPolicyPixels};
  ArrXf blurred{kPolicyPixels};
  for (int row{}; row < static_cast<int>(kPolicyHeight); ++row) {
    for (int col{}; col < static_cast<int>(kPolicyWidth); ++col) {
      float value{};
      for (int offset = -radius; offset <= radius; ++offset) {
        const int source_col = reflect(col + offset, static_cast<int>(kPolicyWidth));
        value += kernel[static_cast<std::size_t>(offset + radius)] *
                 frame[static_cast<Eigen::Index>(row * static_cast<int>(kPolicyWidth) + source_col)];
      }
      horizontal[static_cast<Eigen::Index>(row * static_cast<int>(kPolicyWidth) + col)] = value;
    }
  }
  for (int row{}; row < static_cast<int>(kPolicyHeight); ++row) {
    for (int col{}; col < static_cast<int>(kPolicyWidth); ++col) {
      float value{};
      for (int offset = -radius; offset <= radius; ++offset) {
        const int source_row = reflect(row + offset, static_cast<int>(kPolicyHeight));
        value += kernel[static_cast<std::size_t>(offset + radius)] *
                 horizontal[static_cast<Eigen::Index>(source_row * static_cast<int>(kPolicyWidth) + col)];
      }
      blurred[static_cast<Eigen::Index>(row * static_cast<int>(kPolicyWidth) + col)] = value;
    }
  }
  frame = std::move(blurred);
}

bool DepthHistorySource::readyLocked() const { return history_.size() == kHistoryFrames; }

STEPIT_REGISTER_MODULE(depth_history_source, kDefPriority, Module::make<DepthHistorySource>);
STEPIT_REGISTER_FIELD_SOURCE(depth_history, kDefPriority, Module::make<DepthHistorySource>);
}  // namespace stepit::neuro_policy
