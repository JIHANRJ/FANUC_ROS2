# Motion

Motion docs are now arranged per code piece so each module has its own clean reference.

## Joint (modularized first)

- [docs/joint/readme.md](docs/joint/readme.md)
- [docs/joint/joint_motion.md](docs/joint/joint_motion.md)
- [docs/joint/move_joint.md](docs/joint/move_joint.md)

## Speed (modularized)

- [docs/speed/readme.md](docs/speed/readme.md)
- [docs/speed/speed_control.md](docs/speed/speed_control.md)
- [docs/speed/speed_scaling.md](docs/speed/speed_scaling.md)
- [docs/speed/modular_speed_demo.md](docs/speed/modular_speed_demo.md)

## Standalone commands (no `cd` / `source` prefix)

Assumes your ROS environment is already sourced.

- `ros2 launch fanuc_tools move_joint.launch.py use_mock:=true use_rviz:=false`
- `ros2 launch fanuc_tools move_cartesian.launch.py use_mock:=true use_rviz:=false`
- `ros2 launch fanuc_tools move_linear.launch.py use_mock:=true use_rviz:=false`
- `ros2 run fanuc_tools speed_scaling`
- `ros2 run fanuc_tools modular_speed_demo`
- `ros2 run fanuc_tools collaborative_speed`

## Next pieces to modularize

- `move_cartesian.py`
- `move_linear.py`
- `collaborative_speed.py`
