# policy_neuro_ros2

StepIt plugin for ROS2-based modules that subscribe to ROS2 topics and feed data into the StepIt neuro policy.

### Prerequisites

```shell
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-grid-map-ros
```

### Provided Factories

`stepit::neuro_policy::Module`:

- `cmd_height_subscriber`: subscribes to a ROS2 topic of one of the following types and provides command height field (`cmd_height`):
    - `std_msgs/msg/Float32`,
    - `geometry_msgs/msg/Twist` (`linear.z` component),
    - `geometry_msgs/msg/TwistStamped` (`linear.z` component).
- `cmd_pitch_subscriber`: subscribes to a ROS2 topic of one of the following types and provides command pitch field (`cmd_pitch`):
    - `std_msgs/msg/Float32`,
    - `geometry_msgs/msg/Twist` (`angular.y` component),
    - `geometry_msgs/msg/TwistStamped` (`angular.y` component).
- `cmd_roll_subscriber`: subscribes to a ROS2 topic of one of the following types and provides command roll field (`cmd_roll`):
    - `std_msgs/msg/Float32`,
    - `geometry_msgs/msg/Twist` (`angular.x` component),
    - `geometry_msgs/msg/TwistStamped` (`angular.x` component).
- `cmd_vel_subscriber`: subscribes to a ROS2 topic of one of the following types and provides command velocity fields (`cmd_vel`) with `linear.x`, `linear.y`, and `angular.z` components:
    - `geometry_msgs/msg/Twist`,
    - `geometry_msgs/msg/TwistStamped`.
- `field_subscriber`: subscribes to configured `std_msgs/msg/Float32MultiArray` topics and provides the named fields declared in its config map.
- `heightmap_subscriber`: subscribes to an elevation map topic of type `grid_map_msgs/msg/GridMap` and a pose topic of one of the following types, sample elevation and uncertainty values around the robot, and provide corresponding fields (`heightmap` / `heightmap_uncertainty`):
    - `geometry_msgs/msg/PoseStamped`,
    - `geometry_msgs/msg/PoseWithCovarianceStamped`,
    - `nav_msgs/msg/Odometry`.
- `depth_history_source`: subscribes to a continuous `sensor_msgs/msg/Image` (`32FC1`) or `std_msgs/msg/Float32MultiArray` raw depth stream and provides `depth_history` (2304D). Each received frame is cropped, clipped, and normalized before entering the source-owned four-frame history cache. The policy starts only when four valid, fresh frames have already been buffered; output order is oldest to newest. The deployed G1 depth-student contract is raw 64x36 -> crop `(top=18, bottom=0, left=16, right=16)` -> clip `[0, 2.5]` -> normalize `[0, 1]` -> 18x32.
- `wbc_command_source`: subscribes to one 22D `std_msgs/msg/Float32MultiArray` and provides `wbc_command` in the order `[vx, vy, wz, base_height, left_pose_9d, right_pose_9d]`. Height is absolute metres and clamped to the configured `[0.3, 0.8]` range. Hand poses are pelvis/base-frame position + tangent + normal. A missing hand is exactly nine `NaN` values and holds the most recent valid target; zero poses are not used.

完整的中文话题、22D 布局和发布命令示例见
[`doc/wbc_command_zh.md`](../../doc/wbc_command_zh.md)。

For joystick/external switching, set `teleop.enabled: true` and
`wbc_command_subscriber.default_enabled: false`. The policy starts in joystick
teleop mode. It still receives and validates `/wbc_command`, but does not use
it until the operator presses `LB+Y` (`L1+Y` on the Unitree remote). Pressing
the same combination toggles back to joystick teleop. `LT+B` (`L2+B`) remains
the separate Agent policy on/off control. Set `teleop.enabled: false` only for
an external-command-only deployment. The joystick bridge publishes its 12D
packet on `teleop_command` and, when configured, a 22D preview on a separate
`teleop_wbc_command` topic; it must not publish back to the external
`wbc_command` input topic.

Switching from joystick to external WBC seeds the external command with the
current joystick-generated 22D target, including height and both hand targets.
After at least one valid external message arrives, a publishing gap only logs a
timeout warning: the last valid external command remains active until the mode
is switched back to joystick.

The equivalent control channel is `Policy/WbcCommand` with
`EnableSubscriber`, `DisableSubscriber`, or `SwitchSubscriber`.


### Control Commands

- Channel: `Policy/WbcCommand`

  | Action              | Argument | Description                                      |
  | :------------------ | :------- | :----------------------------------------------- |
  | `EnableSubscriber`  |          | Switches the actor to external 22D WBC.          |
  | `DisableSubscriber` |          | Returns to joystick teleop, or the configured default when teleop is disabled. |
  | `SwitchSubscriber`  |          | Toggles joystick/external WBC control.            |

- Channel: `Policy/CmdVel`

  | Action              | Argument | Description                           |
  | :------------------ | :------- | :------------------------------------ |
  | `EnableSubscriber`  |          | Enables subscription to ROS2 topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS2 topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS2 topics.  |

- Channel: `Policy/CmdRoll`

  | Action              | Argument | Description                           |
  | :------------------ | :------- | :------------------------------------ |
  | `EnableSubscriber`  |          | Enables subscription to ROS2 topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS2 topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS2 topics.  |

- Channel: `Policy/CmdPitch`

  | Action              | Argument | Description                           |
  | :------------------ | :------- | :------------------------------------ |
  | `EnableSubscriber`  |          | Enables subscription to ROS2 topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS2 topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS2 topics.  |

- Channel: `Policy/CmdHeight`

  | Action              | Argument | Description                           |
  | :------------------ | :------- | :------------------------------------ |
  | `EnableSubscriber`  |          | Enables subscription to ROS2 topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS2 topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS2 topics.  |

- Channel: `Policy/Heightmap`

  | Action              | Argument | Description                           |
  | :------------------ | :------- | :------------------------------------ |
  | `EnableSubscriber`  |          | Enables subscription to ROS2 topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS2 topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS2 topics.  |


### Joystick Key Bindings

| Key        | Command                             |
| :--------- | :---------------------------------- |
| **LB + A** | `Policy/CmdVel/SwitchSubscriber`    |
| **LB + A** | `Policy/CmdRoll/SwitchSubscriber`   |
| **LB + A** | `Policy/CmdPitch/SwitchSubscriber`  |
| **LB + A** | `Policy/CmdHeight/SwitchSubscriber` |
| **LB + B** | `Policy/Heightmap/SwitchSubscriber` |
| **LB + Y** | `Policy/WbcCommand/SwitchSubscriber` |


### Notes

- Auto-resolution prefers `cmd_vel_subscriber` over the base `cmd_vel_source` field source because it has a higher priority. To force the non-ROS2 source, explicitly add `cmd_vel_source` to `modules:`. Likewise, use `cmd_roll_source`, `cmd_pitch_source`, `cmd_height_source`, or `dummy_heightmap_source` to bypass the ROS2 subscribers for those fields.
- For the G1 WBC depth-student actor, configure `input_fields` in this exact order: `ang_vel` (3), `gravity` (3), `wbc_command` (22), `joint_pos` (29), `joint_vel` (29), `last_action` (29), `depth_history` (2304). The total must be 2419.
