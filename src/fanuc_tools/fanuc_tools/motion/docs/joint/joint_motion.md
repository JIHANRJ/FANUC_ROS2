# joint_motion.py

Reusable MoveIt joint-goal module for use in any ROS 2 node.

## Run context

This file is a library (not a standalone node). Use it by importing into your node.

## Quick import example

```python
from fanuc_tools.motion.core.joint_motion import JointMotionClient

motion = JointMotionClient(
    node,
    planning_group='manipulator',
    vel=0.2,
    acc=0.2,
)

motion.wait_for_server()
motion.send_goal_degrees([0.0, -20.0, 35.0, 0.0, 10.0, 0.0])
```

## Function catalogue

| Function | Description | Input | Sample | Returns |
|---|---|---|---|---|
| `degrees_to_radians()` | Converts joint values from degrees to radians. | `values_deg: Sequence[float]` | `degrees_to_radians([30, -20, 35, 0, 10, 0])` | `list[float]` |
| `radians_to_degrees()` | Converts joint values from radians to degrees. | `values_rad: Sequence[float]` | `radians_to_degrees([0.52, -0.35, 0.61, 0, 0.17, 0])` | `list[float]` |
| `named_joint_map()` | Creates `{joint_name: value}` mapping from a joint vector. | `joint_values`, `joint_names=DEFAULT_JOINT_NAMES` | `named_joint_map([0, 1, 2, 3, 4, 5])` | `dict[str, float]` |
| `build_joint_constraints()` | Builds MoveIt joint constraints from a target vector. | `target_joints`, optional tolerances/names | `build_joint_constraints(target)` | `moveit_msgs.msg.Constraints` |
| `build_move_group_goal()` | Builds a full MoveGroup goal message. | `target_joints`, planning/scaling options | `build_move_group_goal(target, planning_group='manipulator')` | `moveit_msgs.action.MoveGroup.Goal` |
| `JointMotionClient.wait_for_server()` | Waits until `/move_action` is available. | `timeout_sec: float \| None = None` | `motion.wait_for_server(5.0)` | `bool` |
| `JointMotionClient.send_goal()` | Sends radians target asynchronously. | `target_joints`, optional callbacks | `motion.send_goal([0.1, 0.2, 0.3, 0, 0, 0])` | goal send future |
| `JointMotionClient.send_goal_degrees()` | Converts degrees target and sends asynchronously. | `target_joints_deg`, optional callbacks | `motion.send_goal_degrees([10, -15, 20, 0, 5, 0])` | goal send future |

## JointMotionClient constructor

| Argument | Description | Default |
|---|---|---|
| `planning_group` | MoveIt group name. | `manipulator` |
| `vel` | Max velocity scaling `0.0-1.0`. | `0.1` |
| `acc` | Max acceleration scaling `0.0-1.0`. | `0.1` |
| `action_name` | MoveIt action endpoint. | `/move_action` |
| `joint_names` | Ordered joint names list. | `('J1','J2','J3','J4','J5','J6')` |
| `tolerance_above` | Allowed positive joint error. | `0.01` |
| `tolerance_below` | Allowed negative joint error. | `0.01` |
| `constraint_weight` | MoveIt constraint weight. | `1.0` |
| `plan_only` | Plan without execution if true. | `False` |
