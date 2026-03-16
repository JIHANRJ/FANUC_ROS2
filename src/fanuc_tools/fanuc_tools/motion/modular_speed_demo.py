"""
modular_speed_demo.py
=====================
Simple learning example for modular speed-scaling helpers.

Run with any motion node active:
    ros2 run fanuc_tools modular_speed_demo
"""

import rclpy
from rclpy.node import Node

from fanuc_tools.motion.speed_control import SCALING_TOPIC, SpeedScalingClient, clamp_speed


def parse_speed_sequence(sequence_text: str) -> list[float]:
    """Parse comma-separated speed values into a clamped float list."""
    values = []
    for raw in sequence_text.split(','):
        token = raw.strip()
        if not token:
            continue
        values.append(clamp_speed(float(token)))
    return values


class ModularSpeedDemoNode(Node):
    """Publishes a short speed sequence to demonstrate modular API usage."""

    def __init__(self):
        super().__init__('modular_speed_demo_node')

        self.declare_parameter('topic', SCALING_TOPIC)
        self.declare_parameter('step_interval', 1.0)
        self.declare_parameter('sequence', '100,70,40,0,40,70,100')

        topic = self.get_parameter('topic').value
        self.step_interval = float(self.get_parameter('step_interval').value)
        sequence_text = self.get_parameter('sequence').value
        self.sequence = parse_speed_sequence(sequence_text)

        if not self.sequence:
            self.sequence = [100.0]

        self.speed_client = SpeedScalingClient(self, topic=topic, initial_speed=self.sequence[0])
        self.index = 0

        self.get_logger().info('Modular speed demo started.')
        self.get_logger().info(f'Topic         : {topic}')
        self.get_logger().info(f'Step interval : {self.step_interval}s')
        self.get_logger().info(f'Sequence      : {self.sequence}')

        self.create_timer(self.step_interval, self.tick)

    def tick(self):
        """Publish next sequence item and stop when done."""
        if self.index >= len(self.sequence):
            self.get_logger().info('Speed sequence finished. Shutting down.')
            rclpy.shutdown()
            return

        speed = self.sequence[self.index]
        speed_value = self.speed_client.set_speed(speed, publish=True)
        self.get_logger().info(
            f'Published speed[{self.index}] = {speed_value}% (Int32={self.speed_client.controller.to_int()})'
        )
        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = ModularSpeedDemoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped by user.')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    main()
