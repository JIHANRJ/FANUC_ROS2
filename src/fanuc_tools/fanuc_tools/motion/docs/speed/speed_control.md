# speed_control.py

Reusable modular speed helpers for import into any ROS node.

## Quick import example

```python
from fanuc_tools.motion.core.speed_control import SpeedScalingClient

speed = SpeedScalingClient(node, topic='/speed_scaling_factor', initial_speed=100.0)
speed.set_speed(50.0, publish=True)
```

## Function catalogue

| Function | Description | Input | Sample | Returns |
|---|---|---|---|---|
| `clamp_speed()` | Clamps speed to a valid range. | `value`, optional min/max | `clamp_speed(120)` | `float` in range 0..100 |
| `speed_to_int()` | Converts speed to rounded `Int32` data value. | `value: float` | `speed_to_int(33.6)` | `int` |
| `SpeedScalingController.set_speed()` | Sets speed state with clamping. | `value: float` | `controller.set_speed(80)` | `float` |
| `SpeedScalingController.increase()` | Increases speed by step. | `step: float = 5.0` | `controller.increase(10)` | `float` |
| `SpeedScalingController.decrease()` | Decreases speed by step. | `step: float = 5.0` | `controller.decrease(10)` | `float` |
| `SpeedScalingController.pause()` | Sets speed to zero. | none | `controller.pause()` | `float` |
| `SpeedScalingController.full_speed()` | Sets speed to 100%. | none | `controller.full_speed()` | `float` |
| `SpeedScalingController.get_speed()` | Reads current speed state. | none | `controller.get_speed()` | `float` |
| `SpeedScalingController.to_int()` | Reads current speed as rounded int. | none | `controller.to_int()` | `int` |
| `SpeedScalingClient.set_speed()` | Updates speed and optionally publishes now. | `value`, `publish=False` | `client.set_speed(40, publish=True)` | `float` |
| `SpeedScalingClient.publish_current()` | Publishes current speed to topic. | none | `client.publish_current()` | published `int` value |

## Constructor arguments

| Argument | Description | Default |
|---|---|---|
| `topic` | Speed scaling topic name. | `/speed_scaling_factor` |
| `initial_speed` | Initial speed percentage. | `100.0` |
| `qos_depth` | Publisher queue depth. | `10` |
