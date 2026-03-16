# Speed Scaling Docs

This section covers speed scaling in modular and standalone forms.

## Files

- [speed_control.md](speed_control.md) — reusable modular speed helpers
- [speed_scaling.md](speed_scaling.md) — standalone interactive speed node
- [modular_speed_demo.md](modular_speed_demo.md) — simple teaching/demo script

## Run commands (no setup prefix)

Assumes ROS environment is already sourced.

- `ros2 run fanuc_tools speed_scaling`
- `ros2 launch fanuc_tools speed_scaling.launch.py`
- `ros2 run fanuc_tools modular_speed_demo`

## Modular function tests

- `python3 -m pytest src/fanuc_tools/test/test_speed_scaling_modular.py -q`

## Notes

- `speed_scaling.py` now uses modular helpers from `speed_control.py`.
- `speed_scaling.launch.py` does not require YAML.
- Optional custom params can still be passed with `--ros-args --params-file`.
