"""Interactive joint and Cartesian point recorder and sequencer for FANUC MoveIt.

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
from tf2_ros import Buffer, TransformListener

from fanuc_tools.motion.core.joint_motion import (
    DEFAULT_JOINT_NAMES,
    GOAL_REJECTED_ERROR_CODE,
    JointMotionClient,
    radians_to_degrees,
)


@dataclass
class CartesianPose:
    """Cartesian pose captured from TF for the recorded waypoint."""

    target_frame: str
    source_frame: str
    position_m: list[float]
    orientation_xyzw: list[float]


@dataclass
class RecordedPoint:
    """One named waypoint captured from the current robot state."""

    name: str
    joints_rad: list[float]
    cartesian_pose: CartesianPose | None = None
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
        base_frame: str,
        ee_frame: str,
    ):
        super().__init__('joint_sequence_demo')

        self.sequence_file = sequence_file
        self.pause_sec = float(pause_sec)
        self.base_frame = str(base_frame)
        self.ee_frame = str(ee_frame)
        self.ee_frame_fallbacks = ('pointer_tcp', 'end_effector', 'ee_link', 'flange')
        self.current_joints: list[float] | None = None
        self.points: list[RecordedPoint] = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.motion_client = JointMotionClient(
            self,
            planning_group='manipulator',
            vel=vel,
            acc=acc,
        )

        self.create_subscription(JointState, '/joint_states', self._joint_state_callback, 10)
        self.get_logger().info(
            f'Joint sequence demo ready. Recording joints and Cartesian pose '
            f'({self.base_frame} -> {self.ee_frame}).'
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        name_to_position = dict(zip(msg.name, msg.position))
        self.current_joints = [name_to_position.get(name, 0.0) for name in DEFAULT_JOINT_NAMES]

    def record_current_point(self, name: str | None = None) -> RecordedPoint | None:
        if self.current_joints is None:
            self.get_logger().warn('No joint state received yet. Wait for /joint_states.')
            return None

        cartesian_pose = self._capture_cartesian_pose()
        if cartesian_pose is None:
            self.get_logger().warn(
                f'No transform available from {self.base_frame} to {self.ee_frame}. '
                'Wait for TF and try again.'
            )
            return None

        point_name = name or f'P{len(self.points) + 1}'
        point = RecordedPoint(
            name=point_name,
            joints_rad=list(self.current_joints),
            cartesian_pose=cartesian_pose,
            pause_sec=self.pause_sec,
        )
        self.points.append(point)

        joints_deg = [round(value, 2) for value in radians_to_degrees(point.joints_rad)]
        cartesian = point.cartesian_pose
        self.get_logger().info(
            f'Recorded {point.name}: joints={joints_deg} deg, '
            f'cartesian=[{cartesian.position_m[0]:.4f}, {cartesian.position_m[1]:.4f}, '
            f'{cartesian.position_m[2]:.4f}] m'
        )
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
            pose = point.cartesian_pose
            if pose is None:
                pose_text = 'cartesian=<missing>'
            else:
                pose_text = (
                    'cartesian=['
                    f'{pose.position_m[0]:.4f}, {pose.position_m[1]:.4f}, {pose.position_m[2]:.4f}'
                    ', '
                    f'{pose.orientation_xyzw[0]:.4f}, {pose.orientation_xyzw[1]:.4f}, '
                    f'{pose.orientation_xyzw[2]:.4f}, {pose.orientation_xyzw[3]:.4f}]'
                )
            self.get_logger().info(
                f'  {index}. {point.name} -> joints={joints_deg} deg, {pose_text}, '
                f'pause {point.pause_sec:.1f}s'
            )

    def clear_points(self) -> None:
        self.points.clear()
        self.get_logger().info('Recorded points cleared.')

    def save_points(self, path: Path | None = None) -> Path:
        save_path = path or self.sequence_file
        save_path.parent.mkdir(parents=True, exist_ok=True)

        payload = {
            'joint_names': list(DEFAULT_JOINT_NAMES),
            'base_frame': self.base_frame,
            'ee_frame': self.ee_frame,
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
            cartesian_pose_data = entry.get('cartesian_pose')
            cartesian_pose = None
            if cartesian_pose_data is not None:
                cartesian_pose = CartesianPose(
                    target_frame=str(cartesian_pose_data['target_frame']),
                    source_frame=str(cartesian_pose_data['source_frame']),
                    position_m=[float(value) for value in cartesian_pose_data['position_m']],
                    orientation_xyzw=[float(value) for value in cartesian_pose_data['orientation_xyzw']],
                )
            loaded_points.append(
                RecordedPoint(
                    name=str(entry['name']),
                    joints_rad=[float(value) for value in entry['joints_rad']],
                    cartesian_pose=cartesian_pose,
                    pause_sec=float(entry.get('pause_sec', self.pause_sec)),
                )
            )

        self.points = loaded_points
        self.get_logger().info(f'Loaded {len(self.points)} point(s) from {load_path}')
        return load_path

    def _capture_cartesian_pose(self) -> CartesianPose | None:
        target_frames = [self.ee_frame]
        for frame_name in self.ee_frame_fallbacks:
            if frame_name not in target_frames:
                target_frames.append(frame_name)

        for source_frame in target_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    source_frame,
                    rclpy.time.Time(),
                )
            except Exception:
                continue

            if source_frame != self.ee_frame:
                self.get_logger().warn(
                    f'Using TF frame {source_frame} instead of unavailable {self.ee_frame}.'
                )
                self.ee_frame = source_frame

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return CartesianPose(
                target_frame=self.base_frame,
                source_frame=source_frame,
                position_m=[translation.x, translation.y, translation.z],
                orientation_xyzw=[rotation.x, rotation.y, rotation.z, rotation.w],
            )

        return None

    def replay_points(self) -> None:
        if not self.points:
            self.get_logger().warn('No points to replay.')
            return

        self.motion_client.wait_for_server()
        self.get_logger().info(f'Replaying {len(self.points)} point(s)...')

        total_points = len(self.points)

        for index, point in enumerate(self.points, start=1):
            joints_deg = [round(value, 2) for value in radians_to_degrees(point.joints_rad)]
            if point.cartesian_pose is None:
                pose_text = 'cartesian=<missing>'
            else:
                pose = point.cartesian_pose
                pose_text = (
                    'cartesian=['
                    f'{pose.position_m[0]:.4f}, {pose.position_m[1]:.4f}, {pose.position_m[2]:.4f}'
                    ']'
                )
            self.get_logger().info(
                f'[{index}/{total_points}] Moving to {point.name}: joints={joints_deg} deg, {pose_text}'
            )
            self._send_goal_and_wait(point.joints_rad)

            if point.pause_sec > 0:
                time.sleep(point.pause_sec)

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
    parser = argparse.ArgumentParser(description='Record and replay FANUC joint and Cartesian points.')
    parser.add_argument(
        '--file',
        default='joint_sequence.json',
        help='JSON file used to save/load recorded points.',
    )
    parser.add_argument('--vel', type=float, default=0.05, help='Velocity scaling for replay.')
    parser.add_argument('--acc', type=float, default=0.05, help='Acceleration scaling for replay.')
    parser.add_argument('--pause', type=float, default=1.0, help='Pause between replayed points.')
    parser.add_argument('--base-frame', default='base_link', help='TF target frame for cartesian capture.')
    parser.add_argument('--ee-frame', default='pointer_tcp', help='TF source frame for cartesian capture.')
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
        base_frame=args.base_frame,
        ee_frame=args.ee_frame,
    )

    spinner = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spinner.start()

    try:
        if args.demo:
            node.points = [
                RecordedPoint(
                    name='home',
                    joints_rad=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    cartesian_pose=None,
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
                    cartesian_pose=None,
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
                    cartesian_pose=None,
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
