"""Interactive joint point recorder and sequencer for FANUC MoveIt.

Run this alongside the official mock launch to record joint positions from
RViz, save them to JSON, and replay them as a sequence.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from fanuc_tools.motion.core.joint_motion import (
    DEFAULT_JOINT_NAMES,
    GOAL_REJECTED_ERROR_CODE,
    JointMotionClient,
    radians_to_degrees,
)


@dataclass
class RecordedPoint:
    """One named joint waypoint captured from the current robot state."""

    name: str
    joints_rad: list[float]
    pause_sec: float = 1.0


class JointSequenceDemo(Node):
    """Record, save, load, and replay joint waypoints."""

    def __init__(
        self,
        *,
        sequence_file: Path,
        vel: float,
        acc: float,
        pause_sec: float,
        interpolation_step_deg: float,
    ):
        super().__init__('joint_sequence_demo')

        self.sequence_file = sequence_file
        self.pause_sec = float(pause_sec)
        self.interpolation_step_deg = float(interpolation_step_deg)
        self.current_joints: list[float] | None = None
        self.points: list[RecordedPoint] = []
        self.motion_client = JointMotionClient(
            self,
            planning_group='manipulator',
            vel=vel,
            acc=acc,
        )

        self.create_subscription(JointState, '/joint_states', self._joint_state_callback, 10)
        self.get_logger().info(
            f'Joint sequence demo ready. Interpolation step: '
            f'{self.interpolation_step_deg:.1f} deg'
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        name_to_position = dict(zip(msg.name, msg.position))
        self.current_joints = [name_to_position.get(name, 0.0) for name in DEFAULT_JOINT_NAMES]

    def record_current_point(self, name: str | None = None) -> RecordedPoint | None:
        if self.current_joints is None:
            self.get_logger().warn('No joint state received yet. Wait for /joint_states.')
            return None

        point_name = name or f'P{len(self.points) + 1}'
        point = RecordedPoint(name=point_name, joints_rad=list(self.current_joints), pause_sec=self.pause_sec)
        self.points.append(point)

        joints_deg = [round(value, 2) for value in radians_to_degrees(point.joints_rad)]
        self.get_logger().info(f'Recorded current joint position as {point.name}: {joints_deg} deg')
        return point

    def record_point(self, name: str | None = None) -> RecordedPoint | None:
        """Backward-compatible alias for recording the current robot position."""
        return self.record_current_point(name)

    def list_points(self) -> None:
        if not self.points:
            self.get_logger().info('No recorded points yet.')
            return

        self.get_logger().info('Recorded points:')
        for index, point in enumerate(self.points, start=1):
            joints_deg = [round(value, 2) for value in radians_to_degrees(point.joints_rad)]
            self.get_logger().info(f'  {index}. {point.name} -> {joints_deg} deg, pause {point.pause_sec:.1f}s')

    def clear_points(self) -> None:
        self.points.clear()
        self.get_logger().info('Recorded points cleared.')

    def save_points(self, path: Path | None = None) -> Path:
        save_path = path or self.sequence_file
        save_path.parent.mkdir(parents=True, exist_ok=True)

        payload = {
            'joint_names': list(DEFAULT_JOINT_NAMES),
            'points': [asdict(point) for point in self.points],
        }
        save_path.write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')
        self.get_logger().info(f'Saved {len(self.points)} point(s) to {save_path}')
        return save_path

    def load_points(self, path: Path | None = None) -> Path:
        load_path = path or self.sequence_file
        data = json.loads(load_path.read_text(encoding='utf-8'))

        loaded_points: list[RecordedPoint] = []
        for entry in data.get('points', []):
            loaded_points.append(
                RecordedPoint(
                    name=str(entry['name']),
                    joints_rad=[float(value) for value in entry['joints_rad']],
                    pause_sec=float(entry.get('pause_sec', self.pause_sec)),
                )
            )

        self.points = loaded_points
        self.get_logger().info(f'Loaded {len(self.points)} point(s) from {load_path}')
        return load_path

    def replay_points(self) -> None:
        if not self.points:
            self.get_logger().warn('No points to replay.')
            return

        self.motion_client.wait_for_server()
        self.get_logger().info(f'Replaying {len(self.points)} point(s)...')

        previous_point: RecordedPoint | None = None
        total_points = len(self.points)

        for index, point in enumerate(self.points, start=1):
            if previous_point is None:
                joints_deg = [round(value, 2) for value in radians_to_degrees(point.joints_rad)]
                self.get_logger().info(
                    f'[{index}/{total_points}] Moving to {point.name}: {joints_deg} deg'
                )
                self._send_goal_and_wait(point.joints_rad)
            else:
                self._replay_segment(previous_point, point, index - 1, total_points)

            if point.pause_sec > 0:
                time.sleep(point.pause_sec)

            previous_point = point

    def _replay_segment(
        self,
        start_point: RecordedPoint,
        end_point: RecordedPoint,
        index: int,
        total_points: int,
    ) -> None:
        """Replay a segment using interpolated joint waypoints."""
        start_joints = start_point.joints_rad
        end_joints = end_point.joints_rad
        max_delta_deg = max(
            abs(end_deg - start_deg)
            for start_deg, end_deg in zip(
                radians_to_degrees(start_joints),
                radians_to_degrees(end_joints),
            )
        )

        if max_delta_deg <= 0.0:
            return

        steps = max(2, int(math.ceil(max_delta_deg / self.interpolation_step_deg)))
        self.get_logger().info(
            f'[{index}/{total_points}] Interpolating {start_point.name} -> '
            f'{end_point.name} in {steps} step(s)'
        )

        for step_index in range(1, steps + 1):
            ratio = step_index / steps
            waypoint = [
                start + (end - start) * ratio
                for start, end in zip(start_joints, end_joints)
            ]
            if step_index == steps:
                joints_deg = [round(value, 2) for value in radians_to_degrees(waypoint)]
                self.get_logger().info(
                    f'[{index}/{total_points}] Moving to {end_point.name}: {joints_deg} deg'
                )
            self._send_goal_and_wait(waypoint)

    def _send_goal_and_wait(self, joints_rad: list[float]) -> None:
        done = threading.Event()
        outcome: dict[str, int] = {}

        def on_goal_response(goal_handle) -> None:
            if goal_handle.accepted:
                self.get_logger().info('Goal accepted.')
            else:
                self.get_logger().error('Goal rejected by MoveIt.')

        def on_result(error_code: int, _result) -> None:
            outcome['error_code'] = error_code
            done.set()

        self.motion_client.send_goal(
            joints_rad,
            goal_response_callback=on_goal_response,
            result_callback=on_result,
        )

        while not done.wait(timeout=0.1):
            if not rclpy.ok():
                return

        error_code = outcome.get('error_code', GOAL_REJECTED_ERROR_CODE)
        if error_code == 1:
            self.get_logger().info('Motion complete.')
        else:
            self.get_logger().error(f'Motion failed with error code: {error_code}')


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Record and replay FANUC joint points.')
    parser.add_argument(
        '--file',
        default='joint_sequence.json',
        help='JSON file used to save/load recorded points.',
    )
    parser.add_argument('--vel', type=float, default=0.05, help='Velocity scaling for replay.')
    parser.add_argument('--acc', type=float, default=0.05, help='Acceleration scaling for replay.')
    parser.add_argument('--pause', type=float, default=1.0, help='Pause between replayed points.')
    parser.add_argument(
        '--interp-step-deg',
        type=float,
        default=5.0,
        help='Maximum joint-space step size used to interpolate between points.',
    )
    parser.add_argument(
        '--demo',
        action='store_true',
        help='Load a small built-in joint sequence, save it, and replay it once.',
    )
    return parser


def print_menu() -> None:
    print()
    print('Commands:')
    print('  r - record current joint state as a point')
    print('  l - list recorded points')
    print('  s - save points to JSON')
    print('  o - load points from JSON')
    print('  p - replay points in sequence')
    print('  c - clear recorded points')
    print('  q - quit')


def spin_node(node: Node) -> None:
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    rclpy.init()
    node = JointSequenceDemo(
        sequence_file=Path(args.file).expanduser().resolve(),
        vel=args.vel,
        acc=args.acc,
        pause_sec=args.pause,
        interpolation_step_deg=args.interp_step_deg,
    )

    spinner = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spinner.start()

    try:
        if args.demo:
            node.points = [
                RecordedPoint(
                    name='home',
                    joints_rad=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    pause_sec=args.pause,
                ),
                RecordedPoint(
                    name='pose_a',
                    joints_rad=[
                        math.radians(20.0),
                        math.radians(-15.0),
                        math.radians(30.0),
                        0.0,
                        math.radians(15.0),
                        0.0,
                    ],
                    pause_sec=args.pause,
                ),
                RecordedPoint(
                    name='pose_b',
                    joints_rad=[
                        math.radians(-15.0),
                        math.radians(10.0),
                        math.radians(20.0),
                        0.0,
                        math.radians(-10.0),
                        0.0,
                    ],
                    pause_sec=args.pause,
                ),
            ]
            node.save_points()
            node.replay_points()
            return

        print('Move the robot in RViz, then use the menu to record points.')
        print(f'Current file: {node.sequence_file}')
        print_menu()

        while rclpy.ok():
            command = input('\nEnter command: ').strip().lower()
            if command == 'r':
                name = input('Point name [auto]: ').strip() or None
                node.record_current_point(name)
            elif command == 'l':
                node.list_points()
            elif command == 's':
                node.save_points()
            elif command == 'o':
                path_text = input(f'Load file [{node.sequence_file}]: ').strip()
                node.load_points(Path(path_text).expanduser().resolve() if path_text else None)
            elif command == 'p':
                node.replay_points()
            elif command == 'c':
                node.clear_points()
            elif command == 'q':
                break
            else:
                print_menu()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spinner.join(timeout=2.0)


if __name__ == '__main__':
    main()
