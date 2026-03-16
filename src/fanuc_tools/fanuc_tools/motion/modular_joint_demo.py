"""
modular_joint_demo.py
=====================
Simple learning example for using the modular joint-motion helpers.

Run with MoveIt stack already up (mock or real):
    ros2 run fanuc_tools modular_joint_demo
"""

import time

import rclpy
from rclpy.node import Node

from fanuc_tools.motion.joint_motion import (
    GOAL_REJECTED_ERROR_CODE,
    JointMotionClient,
    degrees_to_radians,
    named_joint_map,
)


class ModularJointDemoNode(Node):
    """One-shot demo that sends a single joint goal using modular helpers."""

    def __init__(self):
        super().__init__('modular_joint_demo_node')

        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('vel', 0.2)
        self.declare_parameter('acc', 0.2)
        self.declare_parameter('startup_delay', 2.0)

        defaults_deg = [0.0, -20.0, 35.0, 0.0, 10.0, 0.0]
        for index, default in enumerate(defaults_deg, start=1):
            self.declare_parameter(f'target_deg.joint_{index}', default)

        self.planning_group = self.get_parameter('planning_group').value
        self.vel = float(self.get_parameter('vel').value)
        self.acc = float(self.get_parameter('acc').value)
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.target_deg = [
            float(self.get_parameter(f'target_deg.joint_{index}').value)
            for index in range(1, 7)
        ]

        self.motion_client = JointMotionClient(
            self,
            planning_group=self.planning_group,
            vel=self.vel,
            acc=self.acc,
        )

        self.start_time = time.monotonic()
        self.sent = False
        self.create_timer(0.2, self.tick)

        self.get_logger().info('Modular joint demo started.')
        self.get_logger().info(f'Planning group : {self.planning_group}')
        self.get_logger().info(f'Vel/Acc scale  : {self.vel}/{self.acc}')
        self.get_logger().info(f'Target (deg)   : {self.target_deg}')

    def tick(self):
        """Send exactly one goal after a short startup delay."""
        if self.sent:
            return

        if time.monotonic() - self.start_time < self.startup_delay:
            return

        target_rad = degrees_to_radians(self.target_deg)
        self.get_logger().info(f'Target (rad)   : {target_rad}')
        self.get_logger().info(f'Target map     : {named_joint_map(target_rad)}')

        if not self.motion_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveIt action server not available.')
            rclpy.shutdown()
            return

        self.sent = True
        self.get_logger().info('Sending modular demo goal...')
        self.motion_client.send_goal(
            target_rad,
            goal_response_callback=self.on_goal_response,
            result_callback=self.on_result,
        )

    def on_goal_response(self, goal_handle):
        """Logs whether the goal was accepted."""
        if goal_handle.accepted:
            self.get_logger().info('Goal ACCEPTED — executing...')
        else:
            self.get_logger().error('Goal REJECTED.')

    def on_result(self, error_code, _result):
        """Logs final result and exits."""
        if error_code == 1:
            self.get_logger().info('Motion completed successfully. ✓')
        elif error_code == GOAL_REJECTED_ERROR_CODE:
            self.get_logger().error('Motion rejected by MoveIt.')
        else:
            self.get_logger().error(f'Motion failed with error code: {error_code}')

        self.get_logger().info('Demo finished. Shutting down.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ModularJointDemoNode()

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
