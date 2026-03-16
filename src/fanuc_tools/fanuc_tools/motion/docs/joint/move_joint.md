# move_joint.py

Standalone joint-loop node that alternates between Position A and Position B.

## Standalone run commands (no setup prefix)

- `ros2 launch fanuc_tools move_joint.launch.py use_mock:=true use_rviz:=false`
- `ros2 run fanuc_tools move_joint`
- `ros2 run fanuc_tools speed_scaling`

## Parameters source

Default parameters come from `move_joint.py` itself.
`move_joint.launch.py` does not require a YAML file.

Optional YAML use (only if you want custom values):

- `ros2 run fanuc_tools move_joint --ros-args --params-file /path/to/move_joint.yaml`

## Function catalogue

| Function | Description | Input | Sample | Returns |
|---|---|---|---|---|
| `declare_move_joint_parameters()` | Declares all ROS parameters used by the node. | `node: Node` | `declare_move_joint_parameters(node)` | `None` |
| `read_joint_vector_degrees()` | Reads `position_a.*` or `position_b.*` values in degrees. | `node: Node`, `prefix: str` | `read_joint_vector_degrees(node, 'position_a')` | `list[float]` |
| `MoveJointLoopConfig.from_node_parameters()` | Builds loop config object from parameters. | `node: Node` | `MoveJointLoopConfig.from_node_parameters(node)` | `MoveJointLoopConfig` |
| `MoveJointNode.send_goal()` | Sends a radians joint vector via `JointMotionClient`. | `target_joints: Sequence[float]` | `node.send_goal(target)` | goal send future |
| `MoveJointNode.goal_response_callback()` | Handles goal acceptance callback. | `goal_handle` | called by client | `None` |
| `MoveJointNode.result_callback()` | Handles execution result and schedules next move. | `error_code: int`, `_result` | called by client | `None` |
| `main()` | Node entrypoint for standalone run. | optional ROS args | `main()` | `None` |

## Notes

- If YAML is used, joint inputs are degrees and conversion to radians is automatic.
- The node is standalone but intentionally thin; motion logic is delegated to `joint_motion.py`.
