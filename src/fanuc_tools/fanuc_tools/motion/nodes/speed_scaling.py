"""
speed_scaling.py
================
Quick run:
    ros2 launch fanuc_tools move_joint.launch.py use_mock:=true use_rviz:=false
    ros2 run fanuc_tools speed_scaling

Example: Dynamically control the FANUC CRX-10iA/L execution speed
         while a trajectory is running.

What it does:
    Continuously reads a speed value (0-100) from terminal input and
    publishes it to /speed_scaling_factor. The robot's
    scaled_joint_trajectory_controller adjusts execution speed in real time.
"""

import threading

import rclpy
from rclpy.node import Node

from fanuc_tools.motion.core.speed_control import (
    MAX_SPEED,
    MIN_SPEED,
    SCALING_TOPIC,
    SpeedScalingClient,
)


class SpeedScalingNode(Node):
    """Standalone interactive speed scaling node."""

    def __init__(self):
        super().__init__('speed_scaling_node')

        self.declare_parameter('topic', SCALING_TOPIC)
        self.declare_parameter('initial_speed', 100.0)
        self.declare_parameter('publish_rate', 10.0)

        topic = self.get_parameter('topic').value
        initial_speed = float(self.get_parameter('initial_speed').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.speed_client = SpeedScalingClient(
            self,
            topic=topic,
            initial_speed=initial_speed,
        )

        self.create_timer(1.0 / publish_rate, self.publish_speed)

        self.get_logger().info('Speed Scaling Node started.')
        self.get_logger().info(f'Publishing to {topic} at {publish_rate} Hz')
        self.get_logger().info(
            f'Initial speed: {self.speed_client.controller.get_speed()}%'
        )
        self.get_logger().info('─' * 40)
        self.get_logger().info('Type a value (0-100) and press Enter to change speed.')
        self.get_logger().info('  0   = pause robot completely')
        self.get_logger().info('  50  = half speed')
        self.get_logger().info('  100 = full speed')
        self.get_logger().info('  q   = quit')
        self.get_logger().info('─' * 40)

        self.input_thread = threading.Thread(
            target=self.read_input,
            daemon=True,
        )
        self.input_thread.start()

    def publish_speed(self):
        """Timer callback that publishes current speed value."""
        self.speed_client.publish_current()

    def read_input(self):
        """Read user input and update current speed state."""
        while rclpy.ok():
            try:
                user_input = input('Speed (0-100): ').strip()

                if user_input.lower() == 'q':
                    self.get_logger().info('Quitting...')
                    rclpy.shutdown()
                    break

                value = float(user_input)

                if value < MIN_SPEED or value > MAX_SPEED:
                    self.get_logger().warn(
                        f'Value {value} out of range. Enter a value between 0 and 100.'
                    )
                    continue

                self.speed_client.set_speed(value)

                if value == 0.0:
                    self.get_logger().info('Robot PAUSED.')
                elif value == 100.0:
                    self.get_logger().info('Robot at FULL speed.')
                else:
                    self.get_logger().info(f'Speed set to {value}%')

            except ValueError:
                self.get_logger().warn(
                    'Invalid input. Enter a number between 0-100 or q to quit.'
                )
            except EOFError:
                break


def main(args=None):
    rclpy.init(args=args)
    node = SpeedScalingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    main()
