# G1 flat teacher deployment template

This template is independent of the depth student policies. It has no depth
subscriber and does not alter student modules or student policy files.

## Bundle contents required before launch

Create a separate policy directory and add:

- `actor.onnx`: exported deterministic teacher actor.
- `actor.yml`: copy from this template unchanged unless the exported ONNX ABI
  differs.
- `policy.yml`: copy `policy.yml.template`, then replace all four 29D actuator
  arrays from the same teacher checkpoint.
- `wbc_command.yml`: copy the existing student WBC command config unchanged.
- `base_lin_vel.yml`, `forward_kinematics.yml`, `heightmap.yml`: copy from this
  template.
- `robot.urdf`: the exact 29-DoF training URDF.

`heightmap` is always 255 zeros, with no map scanner or ROS topic. The module
order produces the teacher observation:

```text
base_lin_vel(3), ang_vel(3), gravity(3), wbc_command(22), joint_pos(29),
joint_vel(29), last_action(29), heightmap(255),
left_current_ee_pose(9), right_current_ee_pose(9)
```

The two current EE pose fields are FK-derived, relative to `pelvis`, and use
`[position xyz, R*X tangent xyz, R*Z normal xyz]`. They are not WBC targets.

For MuJoCo, enable its `/odometry` publisher before starting the policy. The
default `velocity_frame: body` is correct when `Odometry.twist` is expressed in
the `child_frame_id`; set it to `world` only for a publisher whose twist is in
the odometry/world frame.
