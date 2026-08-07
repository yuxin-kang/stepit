# RGB/深度反馈手部控制调试流程（基础版）

本文先定义一个不进入 skill 的人工调试流程，用于验证“看图 → 选择手部动作模板 →
发布 WBC 命令 → 再看图”的闭环。第一阶段只调试手部，不控制腿、底盘或行走。

## 1. 设计原则

- 外部输入仍然是 `/stepit/wbc_command` 的 22D 命令。
- 前四个字段固定为安全值：

  ```text
  vx = 0, vy = 0, wz = 0, base_height = 安全站立高度
  ```

- 只改变左右手的 9D 目标；不做在线的“相机坐标 → pelvis/base 坐标”转换。
- 手部目标使用预先手动调好的 base 坐标动作模板。RGB/深度只负责判断应该选择
  哪个模板以及是否继续，不直接生成三维位姿。
- 第一阶段使用固定场景，例如箱子放在机器人胸前、双手可见的位置；不追求任意
  位置和任意物体的泛化。
- 本文只用于人工闭环调试，不创建或运行正式 skill。

### 抓取先验（双手任务的硬约束）

对于需要双手合拢夹持的物体，动作顺序固定为：

```text
较远 → 粗略靠近 → x/z 对齐 → 双手合拢 → hold
```

- `x/z 对齐` 必须先于双手合拢：先用 RGB/深度判断物体与双手在前后方向
  (`x`) 和高度方向 (`z`) 是否处于同一夹持区域；没有对齐时禁止执行 `close`，
  也不要一边前伸/抬高一边收拢双手。
- 如果物体距离较远，先执行较大的、受限的 `approach` 步骤把双手送到物体附近；
  重新采集确认进入近距离后，再切换到小步 `xz_align` 微调。远距离时直接用
  1–2 cm 的 `close` 步长会导致双手在错误深度合拢。
- `close` 阶段只改变双手相向的夹持量（通常是横向位置），保持已经对齐的
  `x/z` 不变；确认连续帧没有滑动后才进入 `hold`。
- 物体丢失、深度无效或对齐方向不确定时，回到 `hold`/`abort`，不得凭单帧继续
  合拢。本文的先验针对双手夹持；单手抓取另行定义。

## 2. 数据来源

机器人 RealSense 启动方式：

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  depth_module.depth_profile:=640x480x15
```

使用的话题：

```text
RGB   /camera/camera/color/image_raw
Depth /camera/camera/aligned_depth_to_color/image_raw
```

当前 RGB/深度采集流程位于：

```text
机器人脚本：/home/unitree/Workspace/wbc_ws/pic/rgb_capture.py
客户端脚本：/home/kyx/robot/home-safety-humanoid/pic/run_rgb_pipeline.sh
本地图片：  deploy/pic/captures/
```

客户端采集一对新帧：

```bash
cd /home/kyx/robot/home-safety-humanoid/pic
./run_rgb_pipeline.sh
```

随后在本地同步：

```bash
cd /srv/shared/home/kyx/robot/home-safety-humanoid/deploy/pic
./pull_latest_from_client.sh
```

每组数据包含：

```text
rgbd_*.jpg                    RGB 原图
rgbd_*_depth.png              原始 16UC1 深度（毫米）
rgbd_*_depth_preview.jpg      深度伪彩预览
rgbd_*.json                   话题、编码、尺寸和时间戳
```

## 3. 22D 命令的最小约束

完整布局见 [`wbc_command_zh.md`](wbc_command_zh.md)：

```text
[vx, vy, wz, base_height,
 left_position(3), left_tangent(3), left_normal(3),
 right_position(3), right_tangent(3), right_normal(3)]
```

虽然本流程只调手部，消息仍必须包含 22 个数。手部方向先固定使用策略包中的
合法默认方向，只调位置。若某只手暂时不动，该手的 9 个字段可全部使用 `NaN`
保持上一目标；不要用 9 个零表示默认手位。

发布频率至少 10 Hz。外部命令超过约 0.25 秒没有更新时，策略会回退安全默认命令。

## 4. 动作模板，而不是坐标转换

先在配置文件或调试脚本中手动填写一组 base 坐标模板。模板的数值来自策略包默认
手位和实机小幅调节，不由相机坐标在线计算。

建议最小模板集合：

| 模板 | 作用 | 初始动作 |
| --- | --- | --- |
| `nominal` | 保持站立时的默认手位 | 左右手使用默认目标 |
| `pregrasp` | 双手张开到箱子两侧 | 左右手分别向外移动约 1–2 cm |
| `approach` | 物体较远时的粗略靠近 | 只沿安全的前伸/高度方向大步靠近，完成后重新看图 |
| `xz_align` | 夹持前对齐前后和高度 | 只修正 `x/z`，不收拢双手 |
| `left_step` | 左手小幅修正 | 只改变左手一个位置字段，步长不超过 2 cm |
| `right_step` | 右手小幅修正 | 只改变右手一个位置字段，步长不超过 2 cm |
| `close` | x/z 已对齐后的双手合拢 | 只改变相向的横向位置，不再改变 `x/z` |
| `hold` | 保持当前夹持姿态 | 两只手位置不再变化 |
| `abort` | 中止当前动作 | 停止发布并返回手柄控制 |

`approach` 可以使用比微调更大的受限步长，但每次动作后必须重新采集；进入
`xz_align` 后再恢复小步调试。除明确的 `xz_align` 阶段外，不要同时改变靠近和
合拢两个因素，避免无法判断是哪一个参数造成了结果变化。

## 5. RGB/深度只用于选择动作

第一阶段由助手人工查看回传帧，按以下规则选择模板：

```text
看不到箱子，或深度大面积无效       → nominal / hold，不继续靠近
箱子明显偏左                       → left_step
箱子明显偏右                       → right_step
箱子位于双手之间但距离较远         → approach（粗略靠近后重新采集）
距离已近但 x 或 z 未对齐             → xz_align（只调 x/z）
x/z 已对齐且需要夹持               → close（只收拢双手）
箱子已经被双手稳定包围             → hold
连续帧中箱子相对双手明显滑动或掉落  → abort
```

深度图在这里主要用于：

- 判断箱子是否有有效深度；
- 判断箱子相对双手是较近还是较远；
- 判断连续帧中箱子是否发生明显位移。

不读取点云，也不把每个像素转换成 pelvis/base 三维坐标。

## 6. 一次人工调试循环

### 6.1 准备

1. 启动 RealSense，确认 RGB 和对齐深度都在发布。
2. 启动 StepIt 策略，先保持手柄控制。
3. 发布端和 StepIt 必须处于同一个 ROS 2/DDS 环境；不要在脚本中硬编码与当前
   运行环境冲突的域设置。
4. 确认机器人站稳，周围没有人和障碍物。

### 6.2 先验证零速度命令

使用 `wbc_command_zh.md` 第 4 节的合法 22D 示例，只把手部值替换成当前模板：

```bash
ros2 topic pub --rate 10 \
  /stepit/wbc_command \
  std_msgs/msg/Float32MultiArray \
  '{data: [0.0, 0.0, 0.0, 0.8,
  <left_9d_values>,
  <right_9d_values>]}'
```

尖括号只是占位符，不能原样发送。先检查：

```bash
ros2 topic info -v /stepit/wbc_command
ros2 topic echo --once /stepit/wbc_command
```

然后按 `L1+Y` 切换到外部 WBC 控制；日志应出现：

```text
Started subscribing external WBC command.
```

### 6.3 每次只做一个小动作

1. 采集并查看一组 RGB/深度。
2. 按 `approach → xz_align → close` 的先验顺序选择动作；每个阶段只能在复拍确认
   后进入下一阶段。
3. 以 10 Hz 连续发布该模板 2–3 秒。
4. 停止发布或保持 `hold`，再次采集一组图片。
5. 比较箱子、双手和深度预览，记录动作是否使箱子更接近目标位置。
6. `approach` 允许先用较大但受限的步长；进入 `xz_align` 和 `close` 后，每次只
   把相关手部位置改动约 1–2 cm，然后重复测试。

第一阶段只做到 `hold`，不要自动抬箱子、行走或改变躯干姿态。确认多组连续帧中
箱子没有滑落后，再单独讨论抬升动作。

## 7. 中止条件

满足任一条件就停止外部发布，并按 `L1+Y` 返回手柄控制：

- RGB 或深度连续丢失；
- 箱子离开画面或无法确认位置；
- 箱子相对双手快速滑动；
- 手部目标超出手动设定的安全范围；
- 命令发布中断超过 0.25 秒；
- 机器人姿态、接触或周围环境异常。

不要在 StepIt 运行期间并行启动另一个手臂底层控制器。

## 8. 进入正式 skill 前的验收条件

至少满足以下条件后，再把模板和状态机整理进 skill：

1. 同一箱子位置连续重复 3 次，`pregrasp → approach → xz_align → close → hold`
   都能完成；
2. 每次只修改一个模板参数即可解释动作变化；
3. RGB/深度丢失时会停在安全状态；
4. 命令持续发布和手柄切换行为稳定；
5. 已记录每个模板的实际效果和适用范围。

正式 skill 只需把本文的动作表、视觉判定和安全中止条件固化；在此之前全部保留
人工确认，避免调试逻辑直接接管机器人。

## 9. 启发式系统的四个层次

为了避免文档最后变成不断增加 `if/else` 的一次性脚本，把调试流程固定成四层：

```text
观测层 Observation
  RGB、对齐深度、采集时间戳、StepIt 日志

状态层 State
  箱子是否可见、目标偏左/中/右、深度有效/无效、近/远、是否稳定

动作层 Action
  nominal、pregrasp、left_step、right_step、close、hold、abort

反馈层 Feedback
  动作前后图片、深度预览、命令模板、持续时间、结果和失败原因
```

状态层只保留能解释动作的离散变量，不追求从单帧图像恢复完整三维场景。例如：

```text
target_visible : yes / no
target_side    : left / center / right
depth_state    : invalid / far / near
grasp_state    : unknown / between_hands / stable / slipping
```

`grasp_state=stable` 只有在连续多帧中箱子相对双手没有明显滑动时才允许使用；单帧
不能直接把它判定为稳定。

## 10. 动作探针（Action Probe）

每次调试先做一个短、可回放的动作探针，而不是直接运行完整任务。动作探针至少
包含以下字段：

```text
probe_name       探针名称
input_capture    动作前 RGB/深度文件
template         使用的手部模板
duration_s       持续时间，初始 2–3 秒
expected_effect  预期图片变化
output_capture   动作后 RGB/深度文件
outcome          improved / no_change / overshoot / unsafe / aborted
failure_reason   失败原因
next_change      下一轮只改一个参数
```

第一批探针建议固定为：

| 探针 | 模板 | 预期变化 |
| --- | --- | --- |
| `p0_nominal` | `nominal` | 图片和手部保持稳定 |
| `p1_left_sign` | `left_step` | 左手按预期方向移动 |
| `p2_right_sign` | `right_step` | 右手按预期方向移动 |
| `p3_close_small` | `close` | 双手更接近箱子，但不产生明显冲击 |
| `p4_hold` | `hold` | 连续帧中箱子相对双手不滑落 |

如果 `p1_left_sign` 的实际方向相反，只修正 `left_step` 模板的符号或数值，
不要同时修改视觉判断和多个动作模板。

## 11. 试验记录格式

先使用 JSONL 记录每一轮，不要求现在就写记录程序；手工记录也可以。建议格式：

```json
{
  "trial_id": "box_20260806_001",
  "scene": "box_center_fixed",
  "before": {
    "rgb": "rgbd_before.jpg",
    "depth_preview": "rgbd_before_depth_preview.jpg",
    "target_visible": true,
    "target_side": "left",
    "depth_state": "near",
    "grasp_state": "unknown"
  },
  "action": {
    "template": "left_step",
    "duration_s": 2.0,
    "step_cm": 2.0
  },
  "after": {
    "rgb": "rgbd_after.jpg",
    "depth_preview": "rgbd_after_depth_preview.jpg",
    "outcome": "improved"
  },
  "failure_reason": null,
  "next_change": "keep_template"
}
```

每轮只允许一个主要变化：模板位置、模板步长、视觉阈值或持续时间四者选一个。
这样才能知道改动是否真的有效。

## 12. 失败分类与更新顺序

观察到失败时，先分类再修改：

| 失败类型 | 典型现象 | 优先修改位置 |
| --- | --- | --- |
| `detector_false_positive` | 把背景/手臂当成箱子 | ROI、颜色/形态学阈值、深度有效性 |
| `target_lost` | 箱子离开画面后仍继续动作 | 状态机的 `abort` 守卫 |
| `wrong_action_sign` | 左动作实际向右 | 对应手部模板的符号 |
| `no_effect` | 发布命令后画面几乎不变 | 模板步长或外部控制权 |
| `overshoot` | 一步越过目标 | 步长、持续时间、死区 |
| `depth_invalid` | 深度预览大面积黑色 | 只保持/停止，不强行靠近 |
| `slipping` | 箱子相对双手连续滑动 | 进入 `abort`，不要直接增大动作 |
| `command_timeout` | 超过 0.25 秒无新消息 | 发布循环和进程状态 |

更新顺序固定为：

```text
先修安全守卫
  → 再修状态判定
  → 再修动作模板
  → 最后才调持续时间和细小参数
```

不要为了通过一个失败样例不断追加特殊分支。出现三个以上相似分支时，应把它们
合并成一个明确的状态或参数，并保留旧样例作为回归检查。

## 13. 回放和回归检查

正式进入 skill 前，至少建立以下五个“黄金场景”：

```text
golden_box_center       箱子居中、深度有效
golden_box_left         箱子偏左
golden_box_right        箱子偏右
golden_depth_invalid    箱子区域深度无效
golden_box_lost         画面中没有箱子
```

回放检查不接管机器人，只使用已保存的 RGB/深度文件，确认：

1. 箱子不可见或深度无效时不会选择 `close`；
2. 箱子偏左/右时只选择对应一侧的小动作；
3. `hold` 不会改变手部目标；
4. 所有异常场景都能进入 `abort` 或安全保持；
5. 修改新规则后，旧黄金场景的结果没有倒退。

每次出现新的最佳结果，都先简化模板和状态规则，再重新回放全部黄金场景；不要
只保留一个“看起来成功”的现场视频。

## 14. 第一轮具体调试计划

### Round 0：确认基线

- 箱子放在固定位置；
- `nominal` 持续发布；
- 只确认 RGB、深度、话题和手柄/外部模式切换正常；
- 不移动箱子，不抬升。

### Round 1：确认方向

- 单独执行 `left_step`；
- 采集动作前后帧；
- 确认左手实际方向；
- 再单独执行 `right_step`；
- 若方向错误，只修模板符号。

### Round 2：确认靠近与对齐顺序

- 先把箱子放在明显较远的位置，使用 `approach` 做一次受限的大步靠近；
- 进入近距离后使用 `xz_align`，分别确认前后 (`x`) 和高度 (`z`) 对齐；
- 只有 x/z 对齐后才使用 `close`，每次只移动约 1–2 cm；
- 每次动作后重新采集一组 RGB/深度；
- 找到不会明显冲击箱子的最大安全靠近步长，以及可重复的 x/z 对齐阈值。

### Round 3：确认保持

- 进入 `hold`，连续采集至少 3 组图片；
- 观察箱子是否相对双手滑动；
- 出现滑动就记录 `slipping`，停止尝试抬升。

### Round 4：重复性

- 在相同箱子位置重复 3 次；
- 每次保存图片、深度、模板和结果；
- 只有重复结果稳定，才考虑写成正式 skill。

## 15. 与参考文章的对应关系

本文借鉴的是 [Learning Beyond Gradients](https://trinkle23897.github.io/learning-beyond-gradients/)
中“检测 → 状态 → 动作 → 反馈 → 记录 → 修改 → 回放”的工程化思想，尤其参考了
[VizDoom 的纯视觉启发式实现](https://github.com/Trinkle23897/learning-beyond-gradients/blob/main/vizdoom/heuristic_vizdoom_d1_cv.py)。

这不是把 VizDoom 的规则搬到机器人上，也不是训练网络；它只是把视觉反馈和动作
模板组织成可观察、可回放、可回归的调试系统。等本文流程在真实机器人上稳定后，
再把经过验证的模板和状态机整理到正式 skill。
