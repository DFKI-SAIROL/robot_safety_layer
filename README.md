# robot_safety_layer

Root controller for the Franka FR3. Entry point for all user commands (Meta Quest / policy), applies workspace safety checks, arbitrates between teleoperation and policy control, integrates joint velocities from `robot_ik_layer`, and drives the robot.

## Role in the pipeline

```
meta_quest / policy  →  target_pose (service)  →  franka_controller_node
                                                         │
                                          safe_target_pose (topic)
                                                         │
                                                   robot_ik_layer
                                                         │
                                               target_dq (topic)
                                                         │
                                          franka_controller_node  →  target_joint  →  robot
```

## Node: `franka_controller_node`

### Topics & Services

| Direction | Name | Type | Description |
|-----------|------|------|-------------|
| Service | `target_pose` | `SetPoseStamped` | Cartesian target from Meta Quest or any caller |
| Sub | `joint_states` | `JointState` | Own arm joint positions |
| Sub | `/<other_ns>/joint_states` | `JointState` | Other arm joint positions (for collision avoidance) |
| Sub | `target_dq` | `JointState` | Joint velocities from `robot_ik_layer` |
| Sub | `policy_target_dq` | `JointState` | Joint velocities from a learned policy |
| Pub | `safe_target_pose` | `PoseStamped` | Safety-adjusted target forwarded to `robot_ik_layer` |
| Pub | `target_joint` | `JointState` | Integrated position + velocity command to the robot |
| Pub | `safety_vis` | `Marker` | RViz visualization of safety boundaries |