"""Simple teaching demo for modular Cartesian motion with tool callbacks."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node

from fanuc_tools.motion.core.cartesian_motion import CartesianMotionClient, CartesianWaypoint


def gripper_open() -> None:
    """Placeholder tool action."""
    print('[tool] open gripper')


def gripper_close() -> None:
    """Placeholder tool action."""
    print('[tool] close gripper')


class ModularCartesianDemoNode(Node):
    """One-shot pick/place teaching sequence using the modular cartesian client."""

    def __init__(self):
        super().__init__('modular_cartesian_demo_node')
        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('end_effector_link', 'pointer_tcp')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('vel', 0.1)
        self.declare_parameter('acc', 0.1)
        self.declare_parameter('startup_delay', 1.0)

        self.client = CartesianMotionClient(
            self,
            planning_group=str(self.get_parameter('planning_group').value),
            end_effector_link=str(self.get_parameter('end_effector_link').value),
            reference_frame=str(self.get_parameter('reference_frame').value),
            vel=float(self.get_parameter('vel').value),
            acc=float(self.get_parameter('acc').value),
        )

        self.sequence = (
            CartesianWaypoint(
                x=0.45,
                y=0.00,
                z=0.55,
                roll_rad=math.pi,
                pitch_rad=0.0,
                yaw_rad=0.0,
                velocity_mm_s=300.0,
                blend_type='FINE',
                before_move=gripper_open,
            ),
            CartesianWaypoint(
                x=0.45,
                y=0.05,
                z=0.42,
                roll_rad=math.pi,
                pitch_rad=0.0,
                yaw_rad=0.0,
                velocity_mm_s=150.0,
                blend_type='CNT',
                after_move=gripper_close,
            ),
            CartesianWaypoint(
                x=0.60,
                y=0.20,
                z=0.50,
                roll_rad=math.pi,
                pitch_rad=0.0,
                yaw_rad=math.radians(20.0),
                velocity_mm_s=350.0,
                blend_type='FINE',
            ),
            CartesianWaypoint(
                x=0.60,
                y=0.20,
                z=0.42,
                roll_rad=math.pi,
                pitch_rad=0.0,
                yaw_rad=math.radians(20.0),
                velocity_mm_s=150.0,
                blend_type='FINE',
                after_move=gripper_open,
            ),
        )
        self.sequence_index = 0
        self.pending_next_index = 0
        self.motion_in_progress = False
        self.start_time = time.monotonic()
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.create_timer(0.5, self.tick)

    def tick(self) -> None:
        """Dispatch the next waypoint or CNT block after startup delay."""
        if self.motion_in_progress:
            return
        if time.monotonic() - self.start_time < self.startup_delay:
            return
        if self.sequence_index >= len(self.sequence):
            self.get_logger().info('Demo complete. ✓')
            rclpy.shutdown()
            return

        self.motion_in_progress = True
        current_waypoint = self.sequence[self.sequence_index]
        if current_waypoint.normalized_blend_type() == 'CNT':
            end_index = self.sequence_index
            while (
                end_index < len(self.sequence)
                and self.sequence[end_index].normalized_blend_type() == 'CNT'
            ):
                end_index += 1
            self.pending_next_index = end_index
            self.client.send_path(
                self.sequence[self.sequence_index:end_index],
                goal_response_callback=self.goal_response_callback,
                result_callback=self.result_callback,
            )
            return

        self.pending_next_index = self.sequence_index + 1
        self.client.send_goal(
            current_waypoint,
            goal_response_callback=self.goal_response_callback,
            result_callback=self.result_callback,
        )

    def goal_response_callback(self, goal_handle) -> None:
        """Log acceptance state."""
        if getattr(goal_handle, 'accepted', False):
            self.get_logger().info('Demo goal accepted.')
            return
        self.get_logger().error('Demo goal rejected.')

    def result_callback(self, error_code: int, _result) -> None:
        """Advance to the next item after success."""
        if error_code != 1:
            self.get_logger().error(f'Demo motion failed with error code {error_code}.')
            rclpy.shutdown()
            return
        self.sequence_index = self.pending_next_index
        self.motion_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    node = ModularCartesianDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Demo interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
