#ifndef STEPIT_ROBOT_UNITREE2_JOYSTICK_UNITREE2_H_
#define STEPIT_ROBOT_UNITREE2_JOYSTICK_UNITREE2_H_

#include <atomic>
#include <array>
#include <cstdint>
#include <mutex>

#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <stepit/joystick/joystick.h>
#include <stepit/robot/unitree2/common.h>

namespace u2_msg = unitree_go::msg::dds_;
namespace u2_hg_msg = unitree_hg::msg::dds_;

namespace stepit {
namespace joystick {
class Unitree2Joystick final : public Joystick {
  union Keys {
    struct {
      uint8_t R1 : 1;
      uint8_t L1 : 1;
      uint8_t start : 1;
      uint8_t select : 1;
      uint8_t R2 : 1;
      uint8_t L2 : 1;
      uint8_t F1 : 1;
      uint8_t F2 : 1;
      uint8_t A : 1;
      uint8_t B : 1;
      uint8_t X : 1;
      uint8_t Y : 1;
      uint8_t up : 1;
      uint8_t right : 1;
      uint8_t down : 1;
      uint8_t left : 1;
    };

    uint16_t value;
  };

 public:
  Unitree2Joystick();
  bool connected() const override;
  void getState(State &state) override;
  static void updateState(State &state, float lx, float ly, float rx, float ry, uint16_t keys_data);

 private:
  struct RemoteData {
    std::array<std::uint8_t, 2> header;
    std::uint16_t keys;
    float lx;
    float rx;
    float ry;
    float l2;
    float ly;
    std::array<std::uint8_t, 16> reserved;
  };
  static_assert(sizeof(RemoteData) == 40, "G1 wireless_remote ABI must be 40 bytes.");

  void callback(const u2_msg::WirelessController_ *msg);
  void lowStateCallback(const u2_hg_msg::LowState_ *msg);

  u2_sdk::ChannelSubscriberPtr<u2_msg::WirelessController_> sub_;
  u2_sdk::ChannelSubscriberPtr<u2_hg_msg::LowState_> low_state_sub_;
  State state_;
  std::mutex mutex_;
  std::atomic<std::size_t> tick_{};
};
}  // namespace joystick
}  // namespace stepit

#endif  // STEPIT_ROBOT_UNITREE2_JOYSTICK_UNITREE2_H_
