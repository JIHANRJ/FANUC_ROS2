"""Standalone FANUC-style Cartesian loop with FINE and CNT sequencing."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from fanuc_tools.motion.core.cartesian_motion import CartesianMotionClient, CartesianWaypoint


def declare_move_cartesian_parameters(node: Node) -> None:
    """Declare parameters for compatible single-pose and sequence modes."""
    node.declare_parameter('planning_group', 'manipulator')
    node.declare_parameter('end_effector_link', 'pointer_tcp')
    node.declare_parameter('reference_frame', 'world')
    node.declare_parameter('vel', 0.1)
    node.declare_parameter('acc', 0.1)
    node.declare_parameter('eef_step', 0.01)
    node.declare_parameter('jump_threshold', 0.0)
    node.declare_parameter('startup_delay', 1.0)
    node.declare_parameter('delay_between_moves', 2.0)
    node.declare_parameter('trajectory_start_delay', 0.1)
    node.declare_parameter('position_tolerance', 0.001)
    node.declare_parameter('orientation_tolerance', 0.01)
    node.declare_parameter('nominal_speed_mm_s', 500.0)
    node.declare_parameter('position_a_sequence_json', '')
    node.declare_parameter('position_b_sequence_json', '')

    defaults = {
        'position_a': (0.50, 0.00, 0.50, 180.0, 0.0, 0.0),
        'position_b': (0.50, 0.20, 0.35, 180.0, 0.0, 0.0),
    }
    for prefix, values in defaults.items():
        node.declare_parameter(f'{prefix}.x', values[0])
        node.declare_parameter(f'{prefix}.y', values[1])
        node.declare_parameter(f'{prefix}.z', values[2])
        node.declare_parameter(f'{prefix}.roll', values[3])
        node.declare_parameter(f'{prefix}.pitch', values[4])
        node.declare_parameter(f'{prefix}.yaw', values[5])


def read_pose_waypoint(node: Node, prefix: str) -> CartesianWaypoint:
    """Read legacy single-pose parameters and convert them to one FINE waypoint."""
    return CartesianWaypoint(
        x=float(node.get_parameter(f'{prefix}.x').value),
        y=float(node.get_parameter(f'{prefix}.y').value),
        z=float(node.get_parameter(f'{prefix}.z').value),
        roll_rad=math.radians(float(node.get_parameter(f'{prefix}.roll').value)),
        pitch_rad=math.radians(float(node.get_parameter(f'{prefix}.pitch').value)),
        yaw_rad=math.radians(float(node.get_parameter(f'{prefix}.yaw').value)),
        velocity_mm_s=float(node.get_parameter('nominal_speed_mm_s').value),
        blend_type='FINE',
    )


def parse_waypoint_sequence_json(sequence_json: str) -> list[CartesianWaypoint]:
    """Parse a simple JSON waypoint list into `CartesianWaypoint` objects."""
    parsed = json.loads(sequence_json)
    if not isinstance(parsed, list):
        raise ValueError('Waypoint JSON must be a list of waypoint objects.')

    waypoints = []
    for item in parsed:
        if not isinstance(item, dict):
            raise ValueError('Each waypoint entry must be an object.')
        waypoints.append(
            CartesianWaypoint(
                x=float(item['x']),
                y=float(item['y']),
                z=float(item['z']),
                roll_rad=math.radians(float(item.get('roll_deg', 180.0))),
                pitch_rad=math.radians(float(item.get('pitch_deg', 0.0))),
                yaw_rad=math.radians(float(item.get('yaw_deg', 0.0))),
                velocity_mm_s=float(item.get('velocity_mm_s', 500.0)),
                blend_type=str(item.get('blend_type', 'FINE')).upper(),
                blend_radius_m=float(item.get('blend_radius_mm', 100.0)) / 1000.0,
            )
        )
    return waypoints


def load_waypoint_sequence(node: Node, prefix: str) -> list[CartesianWaypoint]:
    """Load sequence JSON when provided, otherwise fall back to one legacy pose."""
    sequence_json = str(node.get_parameter(f'{prefix}_sequence_json').value).strip()
    if sequence_json:
        return parse_waypoint_sequence_json(sequence_json)
    return [read_pose_waypoint(node, prefix)]


@dataclass(frozen=True)
class MoveCartesianLoopConfig:
    """Configuration for the standalone loop node."""

    planning_group: str
    end_effector_link: str
    reference_frame: str
    vel: float
    acc: float
    eef_step: float
    jump_threshold: float
    startup_delay: float
    delay_between_moves: float
    trajectory_start_delay: float
    position_tolerance: float
    orientation_tolerance: float
    nominal_speed_mm_s: float
    position_a_sequence: tuple[CartesianWaypoint, ...]
    position_b_sequence: tuple[CartesianWaypoint, ...]

    @classmethod
    def from_node_parameters(cls, node: Node) -> 'MoveCartesianLoopConfig':
        """Build config from current ROS parameters."""
        return cls(
            planning_group=str(node.get_parameter('planning_group').value),
            end_effector_link=str(node.get_parameter('end_effector_link').value),
            reference_frame=str(node.get_parameter('reference_frame').value),
            vel=float(node.get_parameter('vel').value),
            acc=float(node.get_parameter('acc').value),
            eef_step=float(node.get_parameter('eef_step').value),
            jump_threshold=float(node.get_parameter('jump_threshold').value),
            startup_delay=float(node.get_parameter('startup_delay').value),
            delay_between_moves=float(node.get_parameter('delay_between_moves').value),
            trajectory_start_delay=float(node.get_parameter('trajectory_start_delay').value),
            position_tolerance=float(node.get_parameter('position_tolerance').value),
            orientation_tolerance=float(node.get_parameter('orientation_tolerance').value),
            nominal_speed_mm_s=float(node.get_parameter('nominal_speed_mm_s').value),
            position_a_sequence=tuple(load_waypoint_sequence(node, 'position_a')),
            position_b_sequence=tuple(load_waypoint_sequence(node, 'position_b')),
        )


class MoveCartesianNode(Node):
    """Loop node that executes sequence A then sequence B indefinitely."""

    def __init__(self):
        super().__init__('move_cartesian_node')
        declare_move_cartesian_parameters(self)
        self.config = MoveCartesianLoopConfig.from_node_parameters(self)
        self.client = CartesianMotionClient(
            self,
            planning_group=self.config.planning_group,
            end_effector_link=self.config.end_effector_link,
            reference_frame=self.config.reference_frame,
            vel=self.config.vel,
            acc=self.config.acc,
            nominal_speed_mm_s=self.config.nominal_speed_mm_s,
            eef_step=self.config.eef_step,
            jump_threshold=self.config.jump_threshold,
            trajectory_start_delay=self.config.trajectory_start_delay,
            position_tolerance=self.config.position_tolerance,
            orientation_tolerance=self.config.orientation_tolerance,
        )

        self.current_target = 'A'
        self.iteration = 0
        self.motion_in_progress = False
        self.start_time = time.monotonic()
        self.next_attempt_time = self.start_time + self.config.startup_delay
        self.active_sequence: tuple[CartesianWaypoint, ...] = ()
        self.active_index = 0
        self.pending_next_index = 0

        self.get_logger().info(f'Planning group: {self.config.planning_group}')
        self.get_logger().info(
            f'Sequence A waypoints: {len(self.config.position_a_sequence)} | '
            f'Sequence B waypoints: {len(self.config.position_b_sequence)}'
        )
        self.create_timer(0.5, self.loop_tick)

    def loop_tick(self) -> None:
        """Kick off the next sequence when the current one has finished."""
        if self.motion_in_progress:
            return
        if time.monotonic() < self.next_attempt_time:
            return

        self.motion_in_progress = True
        self.iteration += 1
        self.active_sequence = (
            self.config.position_a_sequence
            if self.current_target == 'A'
            else self.config.position_b_sequence
        )
        self.active_index = 0

        self.get_logger().info('─' * 40)
        self.get_logger().info(
            f'Iteration {self.iteration} — Executing Position {self.current_target}'
        )
        self._dispatch_next_chunk()

    def _dispatch_next_chunk(self) -> None:
        if self.active_index >= len(self.active_sequence):
            self._finish_sequence(success=True)
            return

        current_waypoint = self.active_sequence[self.active_index]
        if current_waypoint.normalized_blend_type() == 'CNT':
            end_index = self.active_index
            while (
                end_index < len(self.active_sequence)
                and self.active_sequence[end_index].normalized_blend_type() == 'CNT'
            ):
                end_index += 1
            cnt_waypoints = list(self.active_sequence[self.active_index:end_index])
            self.pending_next_index = end_index
            self.get_logger().info(
                f'CNT block: waypoints {self.active_index + 1}..{end_index}'
            )
            self.client.send_path(
                cnt_waypoints,
                goal_response_callback=self.goal_response_callback,
                result_callback=self.result_callback,
            )
            return

        self.pending_next_index = self.active_index + 1
        self.get_logger().info(f'FINE waypoint {self.active_index + 1}')
        self.client.send_goal(
            current_waypoint,
            goal_response_callback=self.goal_response_callback,
            result_callback=self.result_callback,
        )

    def goal_response_callback(self, goal_handle) -> None:
        """Log whether MoveIt accepted the current action goal."""
        if getattr(goal_handle, 'accepted', False):
            self.get_logger().info('Goal accepted.')
            return
        self.get_logger().error('Goal rejected.')

    def result_callback(self, error_code: int, _result) -> None:
        """Continue through the sequence or retry after a failure."""
        if error_code != 1:
            self.get_logger().error(f'Motion failed with error code {error_code}.')
            self._finish_sequence(success=False)
            return

        self.active_index = self.pending_next_index
        self._dispatch_next_chunk()

    def _finish_sequence(self, *, success: bool) -> None:
        if success:
            self.get_logger().info(f'Position {self.current_target} complete. ✓')
            self.current_target = 'B' if self.current_target == 'A' else 'A'
        else:
            self.get_logger().info(f'Retrying Position {self.current_target} on next cycle.')
        self.motion_in_progress = False
        self.next_attempt_time = time.monotonic() + self.config.delay_between_moves


def main(args=None):
    rclpy.init(args=args)
    node = MoveCartesianNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Cartesian loop stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
