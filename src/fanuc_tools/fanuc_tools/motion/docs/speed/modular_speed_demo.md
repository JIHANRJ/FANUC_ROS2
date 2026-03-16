# modular_speed_demo.py

Simple teaching script showing modular speed helper usage.

## Run command (no setup prefix)

- `ros2 run fanuc_tools modular_speed_demo`

## What it does

- Parses a speed sequence parameter (comma-separated).
- Uses `SpeedScalingClient` to publish each speed step.
- Logs each published value and exits after sequence completion.

## Function catalogue

| Function | Description | Input | Sample | Returns |
|---|---|---|---|---|
| `parse_speed_sequence()` | Parses and clamps comma-separated speed text. | `sequence_text: str` | `parse_speed_sequence('100,50,0')` | `list[float]` |
| `ModularSpeedDemoNode.tick()` | Publishes next sequence item and advances state. | none | `node.tick()` | `None` |
| `main()` | Entrypoint for standalone execution. | optional ROS args | `main()` | `None` |

## Parameters

| Parameter | Description | Default |
|---|---|---|
| `topic` | Topic to publish speed messages. | `/speed_scaling_factor` |
| `step_interval` | Seconds between sequence points. | `1.0` |
| `sequence` | Comma-separated speed values. | `100,70,40,0,40,70,100` |
