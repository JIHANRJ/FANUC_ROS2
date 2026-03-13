"""
collaborative_speed.py
======================
Example: Monitor the FANUC CRX-10iA/L collaborative speed clamping status.

What it does:
    Subscribes to ~/collaborative_speed_scaling and prints the value
    only when it changes. This tells you whether the robot controller
    is automatically clamping speed for collaborative safety.

When to use it:
    - Debugging collaborative speed clamping behaviour
    - Verifying the robot is in/out of collaborative mode
    - Monitoring safety status during operation

Key concepts:
    - collaborative_speed_scaling: published by fanuc_gpio_controller
    - Value 1 = robot running normally, no clamping active
    - Value 0 = robot speed is being clamped by the safety system
    - Non-collaborative robots always publish 1

Important:
    This topic is only available when the FANUC GPIO controller is running.
    It will not be available in mock hardware mode — use with real hardware.

Topic subscribed:
    /fanuc_gpio_controller/collaborative_speed_scaling
    [fanuc_msgs/msg/CollaborativeSpeedScaling]

Usage:
    ros2 launch fanuc_tools collaborative_speed.launch.py

Or run directly:
    ros2 run fanuc_tools collaborative_speed
"""

import rclpy
from rclpy.node import Node
from fanuc_msgs.msg import CollaborativeSpeedScaling


# Human readable status labels
STATUS_LABELS = {
    1: 'NORMAL   — no speed clamping active',
    0: 'CLAMPED  — robot controller is limiting speed for safety',
}


class CollaborativeSpeedNode(Node):
    """
    Monitors the collaborative speed clamping status published
    by the FANUC GPIO controller and prints it when it changes.
    """

    def __init__(self):
        super().__init__('collaborative_speed_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter(
            'topic',
            '/fanuc_gpio_controller/collaborative_speed_scaling'
        )

        topic = self.get_parameter('topic').value

        # ── State tracking ───────────────────────────────────────────────────
        # Store last value so we only print when something changes
        self.last_value = None

        # ── Subscriber ───────────────────────────────────────────────────────
        self.create_subscription(
            CollaborativeSpeedScaling,
            topic,
            self.callback,
            10
        )

        self.get_logger().info('Collaborative Speed Monitor started.')
        self.get_logger().info(f'Subscribing to: {topic}')
        self.get_logger().info('Waiting for messages...')
        self.get_logger().info(
            'Note: this topic requires real hardware + GPIO controller.'
        )

    def callback(self, msg):
        """
        Called every time a new message arrives on the topic.
        Only prints if the value has changed since last message.
        """
        current_value = msg.collaborative_speed_scaling

        # Only print if value changed
        if current_value == self.last_value:
            return

        # Value changed — print update
        self.last_value = current_value

        label = STATUS_LABELS.get(
            current_value,
            f'UNKNOWN value: {current_value}'
        )

        self.get_logger().info('─' * 50)
        self.get_logger().info(f'Collaborative speed scaling changed:')
        self.get_logger().info(f'  Value  : {current_value}')
        self.get_logger().info(f'  Status : {label}')
        self.get_logger().info('─' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = CollaborativeSpeedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Monitor stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
