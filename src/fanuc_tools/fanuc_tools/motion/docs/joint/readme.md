# Joint Motion Docs

This section covers the joint-motion code in modular and standalone forms.

## Files

- [joint_motion.md](joint_motion.md) — reusable module for importing into other scripts
- [move_joint.md](move_joint.md) — standalone A↔B loop node using the reusable module

## Run commands (no setup prefix)

Assumes ROS environment is already sourced.

- `ros2 launch fanuc_tools move_joint.launch.py use_mock:=true use_rviz:=false`
- `ros2 run fanuc_tools move_joint`
- `ros2 run fanuc_tools modular_joint_demo`
- `ros2 run fanuc_tools speed_scaling`

## Modular function tests

- `python3 -m pytest src/fanuc_tools/test/test_move_joint_modular.py -q`

## Notes

- If `move_joint.yaml` is used, joint inputs are in degrees.
- Internally all motion goals are sent in radians.
- `JointMotionClient` is the preferred integration point for other code.
