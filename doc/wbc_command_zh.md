# StepIt WBC 命令发布教程

本文说明如何通过 ROS 2 向 G1 策略发布 22D WBC command。适用于
`wbc_command_source`，学生策略和教师策略使用相同的命令布局。

## 1. 话题和控制权

| 话题 | 类型 | 方向 | 用途 |
| --- | --- | --- | --- |
| `/stepit/wbc_command` | `std_msgs/msg/Float32MultiArray` | 外部程序 → StepIt | 策略实际使用的 22D 命令 |
| `/stepit/teleop_command` | `std_msgs/msg/Float32MultiArray` | StepIt → ROS 2 | 手柄生成的 12D 控制包，仅用于观察 |
| `/stepit/teleop_wbc_command` | `std_msgs/msg/Float32MultiArray` | StepIt → ROS 2 | 手柄生成的 22D 预览，不会反向输入策略 |

策略启动后默认是手柄控制。外部程序即使持续发布
`/stepit/wbc_command`，命令也只会被缓存，不会立即抢占手柄。切换到外部
控制时，StepIt 会先把当前手柄生成的 22D 命令（包括高度和左右手目标）作为
外部控制的初始命令，因此切换不会跳回默认姿势。

- `L2+B`（StepIt 输入 `LT+B`）：进入/退出策略。
- `L1+Y`（StepIt 输入 `LB+Y`）：切换手柄控制和外部 WBC 控制。
- 再按一次 `L1+Y`：返回手柄控制。

切换到外部控制时，日志应出现：

```text
Started subscribing external WBC command.
```

## 2. 22D 命令布局

```text
[0:3]    vx, vy, wz                         # m/s, m/s, rad/s
[3]      base_height                        # pelvis 高度，单位 m
[4:13]   left_hand_pose_9d
[13:22]  right_hand_pose_9d
```

每只手的 9D 位姿都在 pelvis/base 坐标系中，格式为：

```text
[position_x, position_y, position_z,
 tangent_x, tangent_y, tangent_z,
 normal_x, normal_y, normal_z]
```

它不是世界坐标、欧拉角、关节角或增量。位置和高度使用米制。
方向的 tangent 和 normal 必须是有限、非零且相互正交的向量。

缺省手部目标使用策略包 `wbc_command.yml` 中的默认位姿。不要用 9 个零
表示缺省位姿；如果要保持上一目标，可将某只手的 9 个值全部写为 `NaN`。

## 3. ROS 2 环境

在发布命令的终端执行：

```bash
source /opt/ros/humble/setup.zsh
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

如果发布端和 StepIt 不在同一台机器，还要确认两台机器的 DDS 网络可达，
并保持相同的 `ROS_DOMAIN_ID`。

## 4. 安全的零速度测试

先启动 StepIt 和策略，再按 `L1+Y` 切换到外部 WBC。然后持续发布一个
零速度、0.8 m 高度、合法默认手部目标的命令：

```bash
ros2 topic pub --rate 10 \
  /stepit/wbc_command \
  std_msgs/msg/Float32MultiArray \
  '{data: [0.0, 0.0, 0.0, 0.8,
  0.154558912, 0.196795493, 0.066538759, 0.980066597, 0.054583866, -0.191023931, 0.198061556, -0.193287760, 0.960942984,
  0.154558912, -0.196785510, 0.066538759, 0.980066597, -0.054583866, -0.191023931, 0.198061556, 0.193287760, 0.960942984]}'
```

此命令包含严格的 22 个浮点数。`--rate 10` 很重要：默认超时阈值为
0.25 秒，单次发布只适合检查接收，不适合持续控制。

确认收到命令：

```bash
ros2 topic info -v /stepit/wbc_command
ros2 topic echo --once /stepit/wbc_command
```

## 5. 小幅速度测试

确认零速度命令正常后，只把前三个值改为小速度，例如前进 0.05 m/s：

```bash
ros2 topic pub --rate 10 \
  /stepit/wbc_command \
  std_msgs/msg/Float32MultiArray \
  '{data: [0.05, 0.0, 0.0, 0.8,
  0.154558912, 0.196795493, 0.066538759, 0.980066597, 0.054583866, -0.191023931, 0.198061556, -0.193287760, 0.960942984,
  0.154558912, -0.196785510, 0.066538759, 0.980066597, -0.054583866, -0.191023931, 0.198061556, 0.193287760, 0.960942984]}'
```

首次实机测试应从 `vx=0` 开始，确认站立、PD 和安全限位正常后再增加速度。

## 6. 配置要求

策略包的 `wbc_command.yml` 应包含：

```yaml
wbc_command_subscriber:
  default_enabled: false

teleop:
  enabled: true
  wbc_publisher:
    topic: "teleop_wbc_command"
```

`teleop_wbc_command` 必须和外部输入 `/stepit/wbc_command` 分开，避免手柄
预览消息反馈覆盖外部命令。

## 7. 超时和故障排查

- 外部模式下命令超过 0.25 秒没有新消息，策略只记录一次超时警告，并继续
  保持最后一条有效外部命令；不会跳回默认命令。
- 如果切换到外部模式后还没有收到过有效外部消息，则继续保持切换瞬间的
  手柄命令，直到第一条有效外部消息到达。
- 22D 数量不对、速度/高度不是有限值、手部方向非法时，消息会被拒绝。
- 查看话题：

  ```bash
  ros2 topic list -t | grep -E 'wbc_command|teleop_command'
  ros2 topic info -v /stepit/wbc_command
  ```

- 如果手柄仍能控制，说明当前仍在手柄模式；按 `L1+Y` 后应看到
  `Started subscribing external WBC command.`。
- 停止发布端后，StepIt 仍保持最后一条有效外部命令；按 `L1+Y` 才返回手柄
  控制。不要直接结束 StepIt 进程作为模式切换手段。
