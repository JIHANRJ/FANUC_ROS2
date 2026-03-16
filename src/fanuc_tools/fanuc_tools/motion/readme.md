# Motion

Motion examples and reusable helpers for FANUC ROS 2 motion control.

This folder now has two layers:

- **Reusable API** for importing into other code: `joint_motion.py`
- **Standalone scripts** that still run directly: `move_joint.py`, `move_cartesian.py`, `move_linear.py`, `speed_scaling.py`, `collaborative_speed.py`

---

## Required setup command

Run this before any motion command below:

```bash
cd ~/ws_fanuc && source /opt/ros/humble/setup.bash && source install/setup.bash
```

---

## Standalone commands

### `move_joint.py`

Start MoveIt + looping joint motion:

```bash
cd ~/ws_fanuc && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch fanuc_tools move_joint.launch.py use_mock:=true use_rviz:=false
```

Optional speed control in another terminal:

```bash
cd ~/ws_fanuc && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run fanuc_tools speed_scaling
```

### `move_cartesian.py`

```bash
cd ~/ws_fanuc && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch fanuc_tools move_cartesian.launch.py use_mock:=true use_rviz:=false
```

### `move_linear.py`

```bash
cd ~/ws_fanuc && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch fanuc_tools move_linear.launch.py use_mock:=true use_rviz:=false
```

### `speed_scaling.py`

Use this while one of the motion examples is already running:

```bash
cd ~/ws_fanuc && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run fanuc_tools speed_scaling
```

### `collaborative_speed.py`

Real hardware only:

```bash
cd ~/ws_fanuc && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run fanuc_tools collaborative_speed
```

---

## Reusable modular API

### `joint_motion.py`

This is the importable layer for other code.

**Use from another script:**

```python
from fanuc_tools.motion.joint_motion import JointMotionClient, JointMotionSettings

settings = JointMotionSettings(
    planning_group='manipulator',
    vel=0.2,
    acc=0.2,
)

motion = JointMotionClient(node, settings)
motion.wait_for_server()

motion.send_goal_degrees(
    [0.0, -20.0, 35.0, 0.0, 10.0, 0.0],
    result_callback=lambda error_code, result: print('done', error_code),
)
```

#### `joint_motion.py` — Function Catalogue

| Function | Input | Sample | Returns |
|---|---|---|---|
| `degrees_to_radians()` | `values_deg: Sequence[float]` | `degrees_to_radians([30, -20, 35, 0, 10, 0])` | `list[float]` in radians |
| `radians_to_degrees()` | `values_rad: Sequence[float]` | `radians_to_degrees([0.52, -0.35, 0.61, 0, 0.17, 0])` | `list[float]` in degrees |
| `named_joint_map()` | `joint_values`, `joint_names=DEFAULT_JOINT_NAMES` | `named_joint_map([0, 1, 2, 3, 4, 5])` | `{'J1': 0, 'J2': 1, ...}` |
| `build_joint_constraints()` | `target_joints`, `settings` | `build_joint_constraints(target, settings)` | `moveit_msgs.msg.Constraints` |
| `build_move_group_goal()` | `target_joints`, `settings` | `build_move_group_goal(target, settings)` | `moveit_msgs.action.MoveGroup.Goal` |
| `JointMotionClient.wait_for_server()` | `timeout_sec: float \| None = None` | `motion.wait_for_server(5.0)` | `bool` |
| `JointMotionClient.send_goal()` | `target_joints`, optional callbacks | `motion.send_goal([0.1, 0.2, 0.3, 0, 0, 0])` | send future |
| `JointMotionClient.send_goal_degrees()` | `target_joints_deg`, optional callbacks | `motion.send_goal_degrees([10, -15, 20, 0, 5, 0])` | send future |

#### `JointMotionSettings` — Fields

| Field | Meaning |
|---|---|
| `planning_group` | MoveIt planning group, usually `manipulator` |
| `vel` | velocity scaling factor in range `0.0-1.0` |
| `acc` | acceleration scaling factor in range `0.0-1.0` |
| `action_name` | MoveIt action name, default `/move_action` |
| `joint_names` | ordered joint list, default `J1..J6` |
| `tolerance_above` / `tolerance_below` | per-joint goal tolerance |
| `constraint_weight` | MoveIt joint constraint weight |
| `plan_only` | `True` for planning only, `False` for execute |

---

## Standalone loop API

### `move_joint.py`

`move_joint.py` stays runnable on its own, but now its loop config is also importable.

#### `move_joint.py` — Function Catalogue

| Function | Input | Sample | Returns |
|---|---|---|---|
| `declare_move_joint_parameters()` | `node: Node` | `declare_move_joint_parameters(node)` | `None` |
| `read_joint_vector_degrees()` | `node: Node`, `prefix: str` | `read_joint_vector_degrees(node, 'position_a')` | `list[float]` in degrees |
| `MoveJointLoopConfig.from_node_parameters()` | `node: Node` | `MoveJointLoopConfig.from_node_parameters(node)` | `MoveJointLoopConfig` |
| `MoveJointLoopConfig.motion_settings()` | none | `config.motion_settings()` | `JointMotionSettings` |
| `MoveJointNode.send_goal()` | `target_joints: Sequence[float]` | `node.send_goal(target)` | send future |
| `main()` | optional ROS args | `main()` | `None` |

---

## Other motion modules

### `move_cartesian.py`

Moves between two Cartesian poses through MoveIt pose constraints.

| Function | Input | Sample | Returns |
|---|---|---|---|
| `rpy_to_quaternion()` | `roll_deg`, `pitch_deg`, `yaw_deg` | `rpy_to_quaternion(180, 0, 0)` | `geometry_msgs.msg.Quaternion` |
| `MoveCartesianNode.build_pose_constraints()` | `target_pose: Pose` | `node.build_pose_constraints(pose)` | `moveit_msgs.msg.Constraints` |
| `MoveCartesianNode.send_goal()` | `target_pose: Pose` | `node.send_goal(pose)` | `None` |
| `main()` | optional ROS args | `main()` | `None` |

### `move_linear.py`

Builds a true Cartesian path with `/compute_cartesian_path` and executes it.

| Function | Input | Sample | Returns |
|---|---|---|---|
| `rpy_to_quaternion()` | `roll_deg`, `pitch_deg`, `yaw_deg` | `rpy_to_quaternion(180, 0, 0)` | `geometry_msgs.msg.Quaternion` |
| `MoveLinearNode.get_current_tcp_pose()` | none | `node.get_current_tcp_pose()` | current TCP `Pose` or `None` |
| `MoveLinearNode.request_cartesian_path()` | `waypoints`, `mode` | `node.request_cartesian_path(points, 'square')` | async service future |
| `MoveLinearNode.plan_initial_alignment()` | none | `node.plan_initial_alignment()` | `None` |
| `MoveLinearNode.plan_square_loop()` | none | `node.plan_square_loop()` | `None` |
| `MoveLinearNode.execute_trajectory()` | `trajectory`, `mode` | `node.execute_trajectory(traj, 'square')` | async result future |
| `main()` | optional ROS args | `main()` | `None` |

### `speed_scaling.py`

Publishes integer speed scaling values to `/speed_scaling_factor`.

| Function | Input | Sample | Returns |
|---|---|---|---|
| `SpeedScalingNode.publish_speed()` | none | `node.publish_speed()` | `None` |
| `SpeedScalingNode.read_input()` | none | `node.read_input()` | `None` |
| `main()` | optional ROS args | `main()` | `None` |

### `collaborative_speed.py`

Monitors collaborative speed clamp state from the FANUC GPIO controller.

| Function | Input | Sample | Returns |
|---|---|---|---|
| `CollaborativeSpeedNode.callback()` | `msg: CollaborativeSpeedScaling` | `node.callback(msg)` | `None` |
| `main()` | optional ROS args | `main()` | `None` |

---

## Notes

- Joint values in `move_joint.yaml` are in **degrees** and converted internally to radians.
- Velocity and acceleration scaling use the MoveIt range `0.0-1.0`.
- `speed_scaling.py` publishes `0-100` to `/speed_scaling_factor` as `std_msgs/msg/Int32`.
- `collaborative_speed.py` is for **real hardware**; mock mode will not publish that topic.
- Prefer importing `JointMotionClient` for new code instead of reusing the standalone loop node directly.