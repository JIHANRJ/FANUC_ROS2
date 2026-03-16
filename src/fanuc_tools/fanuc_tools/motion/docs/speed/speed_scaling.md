# speed_scaling.py

Standalone interactive node for live speed control from terminal input.

## Standalone run commands (no setup prefix)

- `ros2 run fanuc_tools speed_scaling`
- `ros2 launch fanuc_tools speed_scaling.launch.py`

## Input behavior

- Enter `0..100` and press Enter to set speed.
- Enter `q` to quit.

## Function catalogue

| Function | Description | Input | Sample | Returns |
|---|---|---|---|---|
| `SpeedScalingNode.publish_speed()` | Timer callback that publishes current speed. | none | `node.publish_speed()` | `None` |
| `SpeedScalingNode.read_input()` | Reads terminal input and updates speed state. | none | `node.read_input()` | `None` |
| `main()` | Entrypoint for standalone execution. | optional ROS args | `main()` | `None` |

## Parameters

| Parameter | Description | Default |
|---|---|---|
| `topic` | Topic to publish speed scaling. | `/speed_scaling_factor` |
| `initial_speed` | Startup speed percentage. | `100.0` |
| `publish_rate` | Publish frequency (Hz). | `10.0` |

## Notes

- This node delegates speed math/publish logic to `speed_control.py`.
- YAML is optional; launch does not require `speed_scaling.yaml`.
