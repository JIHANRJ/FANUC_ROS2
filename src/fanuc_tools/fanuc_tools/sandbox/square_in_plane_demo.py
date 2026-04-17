"""Execute a square path constrained to a plane defined by three recorded points.

By default the script reads `joint_sequence.json`, ignores any point whose name
starts with `zero`, uses the first three remaining points in file order, and
builds a square whose first corner is the first selected point.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from rclpy.node import Node

from fanuc_tools.motion.core.cartesian_motion import (
    CartesianMotionClient,
    CartesianWaypoint,
    GOAL_REJECTED_ERROR_CODE,
    SUCCESS_ERROR_CODE,
    apply_velocity_acceleration_scaling,
    build_cartesian_path_request,
    quaternion_to_rpy,
    prepend_current_state_point,
)
from fanuc_tools.motion.core.joint_motion import JointMotionClient


@dataclass(frozen=True)
class RecordedCartesianPoint:
    """Cartesian point loaded from the recorded joint_sequence.json file."""

    name: str
    joints_rad: tuple[float, ...]
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class PlaneBasis:
    """Orthonormal basis describing the square plane."""

    origin: tuple[float, float, float]
    axis_u: tuple[float, float, float]
    axis_v: tuple[float, float, float]
    normal: tuple[float, float, float]


def _vector_sub(a: Iterable[float], b: Iterable[float]) -> tuple[float, float, float]:
    ax, ay, az = a
    bx, by, bz = b
    return ax - bx, ay - by, az - bz


def _vector_add(a: Iterable[float], b: Iterable[float]) -> tuple[float, float, float]:
    ax, ay, az = a
    bx, by, bz = b
    return ax + bx, ay + by, az + bz


def _vector_scale(vector: Iterable[float], scale: float) -> tuple[float, float, float]:
    x, y, z = vector
    return x * scale, y * scale, z * scale


def _dot(a: Iterable[float], b: Iterable[float]) -> float:
    ax, ay, az = a
    bx, by, bz = b
    return ax * bx + ay * by + az * bz


def _cross(a: Iterable[float], b: Iterable[float]) -> tuple[float, float, float]:
    ax, ay, az = a
    bx, by, bz = b
    return (
        ay * bz - az * by,
        az * bx - ax * bz,
        ax * by - ay * bx,
    )


def _norm(vector: Iterable[float]) -> float:
    x, y, z = vector
    return math.sqrt(x * x + y * y + z * z)


def _normalize(vector: Iterable[float]) -> tuple[float, float, float]:
    length = _norm(vector)
    if length <= 1e-12:
        raise ValueError('Cannot normalize a zero-length vector.')
    x, y, z = vector
    return x / length, y / length, z / length


def _pose_from_recorded_point(point: dict) -> RecordedCartesianPoint:
    pose = point.get('cartesian_pose')
    if not pose:
        raise ValueError(f"Point '{point.get('name')}' does not contain cartesian_pose data.")

    return RecordedCartesianPoint(
        name=str(point['name']),
        joints_rad=tuple(float(value) for value in point['joints_rad']),
        position_m=tuple(float(value) for value in pose['position_m']),
        orientation_xyzw=tuple(float(value) for value in pose['orientation_xyzw']),
    )


def load_recorded_points(sequence_file: Path, point_names: list[str] | None) -> list[RecordedCartesianPoint]:
    data = json.loads(sequence_file.read_text(encoding='utf-8'))
    points = [entry for entry in data.get('points', []) if not str(entry.get('name', '')).startswith('zero')]

    if point_names:
        by_name = {str(entry['name']): entry for entry in points}
        missing = [name for name in point_names if name not in by_name]
        if missing:
            raise ValueError(f"Missing point(s) in {sequence_file}: {', '.join(missing)}")
        ordered_points = [by_name[name] for name in point_names]
    else:
        ordered_points = points[:3]

    if len(ordered_points) < 3:
        raise ValueError('Need at least three non-zero recorded points to define a plane.')

    return [_pose_from_recorded_point(entry) for entry in ordered_points[:3]]


def compute_plane_basis(points: list[RecordedCartesianPoint]) -> PlaneBasis:
    origin = points[0].position_m
    vector_u_raw = _vector_sub(points[1].position_m, origin)
    vector_v_raw = _vector_sub(points[2].position_m, origin)

    axis_u = _normalize(vector_u_raw)
    normal = _normalize(_cross(vector_u_raw, vector_v_raw))
    axis_v = _normalize(_cross(normal, axis_u))

    if _dot(axis_v, vector_v_raw) < 0.0:
        axis_v = _vector_scale(axis_v, -1.0)

    return PlaneBasis(origin=origin, axis_u=axis_u, axis_v=axis_v, normal=normal)


def build_square_waypoints(
    basis: PlaneBasis,
    orientation_xyzw: tuple[float, float, float, float],
    length_m: float,
) -> list[CartesianWaypoint]:
    length = float(length_m)
    origin = basis.origin
    axis_u = basis.axis_u
    axis_v = basis.axis_v

    corners = [
        origin,
        _vector_add(origin, _vector_scale(axis_u, length)),
        _vector_add(
            _vector_add(origin, _vector_scale(axis_u, length)),
            _vector_scale(axis_v, length),
        ),
        _vector_add(origin, _vector_scale(axis_v, length)),
    ]

    roll_rad, pitch_rad, yaw_rad = quaternion_to_rpy(*orientation_xyzw)
    waypoints = [
        CartesianWaypoint(
            x=x,
            y=y,
            z=z,
            roll_rad=roll_rad,
            pitch_rad=pitch_rad,
            yaw_rad=yaw_rad,
            blend_type='FINE',
        )
        for x, y, z in corners
    ]
    waypoints.append(waypoints[0])
    return waypoints


class SquarePlaneDemo(Node):
    """Run a square path lying on the plane defined by three recorded points."""

    def __init__(self, args: argparse.Namespace):
        super().__init__('square_plane_demo')
        self.args = args
        self.sequence_file = Path(args.file).expanduser().resolve()
        self.client = CartesianMotionClient(
            self,
            planning_group=args.planning_group,
            end_effector_link=args.end_effector_link,
            reference_frame=args.reference_frame,
            vel=args.vel,
            acc=args.acc,
            eef_step=args.eef_step,
            jump_threshold=args.jump_threshold,
            trajectory_start_delay=args.trajectory_start_delay,
            position_tolerance=args.position_tolerance,
            orientation_tolerance=args.orientation_tolerance,
        )
        self.joint_client = JointMotionClient(
            self,
            planning_group=args.planning_group,
            vel=args.vel,
            acc=args.acc,
        )

    def _execute_fine_waypoint(self, waypoint: CartesianWaypoint, label: str) -> int:
        done = threading.Event()
        outcome: dict[str, int] = {}

        def on_result(error_code: int, _result) -> None:
            outcome['error_code'] = int(error_code)
            done.set()

        self.get_logger().info(label)
        self.client.send_goal(waypoint, result_callback=on_result)

        while not done.wait(timeout=0.1):
            if not rclpy.ok():
                return GOAL_REJECTED_ERROR_CODE

        return outcome.get('error_code', GOAL_REJECTED_ERROR_CODE)

    def run(self) -> None:
        points = load_recorded_points(self.sequence_file, self.args.point_names)
        basis = compute_plane_basis(points)
        waypoints = build_square_waypoints(basis, points[0].orientation_xyzw, self.args.length)
        locked_orientation = (
            waypoints[0].roll_rad,
            waypoints[0].pitch_rad,
            waypoints[0].yaw_rad,
        )

        self.get_logger().info(
            'Using points: '
            f'{points[0].name}, {points[1].name}, {points[2].name}'
        )
        self.get_logger().info(
            'Plane basis: '
            f'origin={basis.origin}, '
            f'u={basis.axis_u}, v={basis.axis_v}, n={basis.normal}'
        )

        if self.args.dry_run:
            self._print_waypoints(waypoints)
            return

        self.client.wait_for_move_action()
        if self.args.execution_mode == 'cartesian':
            self.client.wait_for_cartesian_path_service()
            self.client.wait_for_execute_action()

        # Step 1: Move to the first corner using the recorded joint waypoint.
        start_done = threading.Event()
        start_outcome: dict[str, int] = {}

        def on_start_result(error_code: int, _result) -> None:
            start_outcome['error_code'] = int(error_code)
            start_done.set()

        self.get_logger().info('Moving to first square corner using recorded joints...')
        self.joint_client.wait_for_server()
        self.joint_client.send_goal(list(points[0].joints_rad), result_callback=on_start_result)

        while not start_done.wait(timeout=0.1):
            if not rclpy.ok():
                return

        if start_outcome.get('error_code', GOAL_REJECTED_ERROR_CODE) != SUCCESS_ERROR_CODE:
            self.get_logger().warn(
                'Joint move to first corner failed '
                f"(error={start_outcome.get('error_code', GOAL_REJECTED_ERROR_CODE)}). "
                'Trying cartesian fallback...'
            )
            fallback_code = self._execute_fine_waypoint(
                waypoints[0],
                'Moving to first square corner using cartesian fallback...',
            )
            if fallback_code != SUCCESS_ERROR_CODE:
                self.get_logger().error(
                    'Failed to reach first square corner with both joint and cartesian methods '
                    f'(error={fallback_code}).'
                )
                return

        if self.args.settle_time > 0.0:
            time.sleep(self.args.settle_time)

        def execute_cartesian_waypoint(target_waypoint: CartesianWaypoint, label: str) -> tuple[int, float]:
            done = threading.Event()
            outcome: dict[str, float] = {}

            request = build_cartesian_path_request(
                [target_waypoint.to_pose()],
                current_joint_state=self.client.current_joint_state,
                planning_group=self.client.planning_group,
                end_effector_link=self.client.end_effector_link,
                reference_frame=self.client.reference_frame,
                eef_step=self.client.eef_step,
                jump_threshold=self.client.jump_threshold,
            )

            def on_cartesian_response(future) -> None:
                try:
                    response = future.result()
                except Exception as exc:
                    self.get_logger().error(f'{label} Cartesian path request failed: {exc}')
                    outcome['error_code'] = float(GOAL_REJECTED_ERROR_CODE)
                    done.set()
                    return

                if response.error_code.val != SUCCESS_ERROR_CODE or response.fraction < 0.999:
                    self.get_logger().error(
                        f'{label} could not be planned completely '
                        f'(fraction={response.fraction:.3f}, code={response.error_code.val}).'
                    )
                    outcome['error_code'] = float(GOAL_REJECTED_ERROR_CODE)
                    outcome['planned_fraction'] = float(response.fraction)
                    done.set()
                    return

                trajectory = response.solution
                apply_velocity_acceleration_scaling(
                    trajectory,
                    velocity_scale=self.client.vel,
                    acceleration_scale=self.client.acc,
                )
                prepend_current_state_point(
                    trajectory,
                    self.client.current_joint_state,
                    start_delay=self.client.trajectory_start_delay,
                )

                goal = ExecuteTrajectory.Goal()
                goal.trajectory = trajectory
                execute_future = self.client.execute_action_client.send_goal_async(goal)
                execute_future.add_done_callback(on_execute_goal_response)

            def on_execute_goal_response(future) -> None:
                try:
                    goal_handle = future.result()
                except Exception as exc:
                    self.get_logger().error(f'{label} ExecuteTrajectory send failed: {exc}')
                    outcome['error_code'] = float(GOAL_REJECTED_ERROR_CODE)
                    done.set()
                    return

                if not getattr(goal_handle, 'accepted', False):
                    self.get_logger().error(f'{label} ExecuteTrajectory goal rejected.')
                    outcome['error_code'] = float(GOAL_REJECTED_ERROR_CODE)
                    done.set()
                    return

                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(on_execute_result)

            def on_execute_result(future) -> None:
                wrapped_result = future.result()
                result = wrapped_result.result
                outcome['error_code'] = float(result.error_code.val)
                done.set()

            self.client.cartesian_path_client.call_async(request).add_done_callback(on_cartesian_response)

            while not done.wait(timeout=0.1):
                if not rclpy.ok():
                    return GOAL_REJECTED_ERROR_CODE, 0.0

            return int(outcome.get('error_code', GOAL_REJECTED_ERROR_CODE)), float(outcome.get('planned_fraction', 1.0))

        def execute_segment(target_waypoint: CartesianWaypoint, segment_index: int) -> tuple[int, float]:
            def execute_refined_step(
                start_waypoint: CartesianWaypoint,
                end_waypoint: CartesianWaypoint,
                step_label: str,
                depth: int = 0,
            ) -> tuple[int, float]:
                error_code, fraction = execute_cartesian_waypoint(end_waypoint, step_label)
                if error_code == SUCCESS_ERROR_CODE:
                    return SUCCESS_ERROR_CODE, 1.0

                distance = math.dist(
                    (start_waypoint.x, start_waypoint.y, start_waypoint.z),
                    (end_waypoint.x, end_waypoint.y, end_waypoint.z),
                )
                if depth >= self.args.max_refine_depth or distance <= self.args.min_refine_step:
                    return error_code, fraction

                midpoint = CartesianWaypoint(
                    x=(start_waypoint.x + end_waypoint.x) / 2.0,
                    y=(start_waypoint.y + end_waypoint.y) / 2.0,
                    z=(start_waypoint.z + end_waypoint.z) / 2.0,
                    roll_rad=locked_orientation[0],
                    pitch_rad=locked_orientation[1],
                    yaw_rad=locked_orientation[2],
                    velocity_mm_s=end_waypoint.velocity_mm_s,
                    blend_type='FINE',
                )

                self.get_logger().warn(
                    f'{step_label} refining segment depth={depth + 1} '
                    f'(distance={distance:.4f} m)...'
                )

                first_code, first_fraction = execute_refined_step(
                    start_waypoint,
                    midpoint,
                    f'{step_label} [refine-A]',
                    depth + 1,
                )
                if first_code != SUCCESS_ERROR_CODE:
                    return first_code, first_fraction * 0.5

                second_code, second_fraction = execute_refined_step(
                    midpoint,
                    end_waypoint,
                    f'{step_label} [refine-B]',
                    depth + 1,
                )
                if second_code != SUCCESS_ERROR_CODE:
                    return second_code, 0.5 + second_fraction * 0.5

                return SUCCESS_ERROR_CODE, 1.0

            if self.args.execution_mode == 'stepwise':
                start_waypoint = waypoints[segment_index - 1]
                edge_length = math.dist(
                    (start_waypoint.x, start_waypoint.y, start_waypoint.z),
                    (target_waypoint.x, target_waypoint.y, target_waypoint.z),
                )
                step_size = max(0.001, float(self.args.step_size))
                step_count = max(1, int(math.ceil(edge_length / step_size)))
                current_waypoint = start_waypoint

                for step_index in range(1, step_count + 1):
                    ratio = step_index / step_count
                    interpolated_waypoint = CartesianWaypoint(
                        x=start_waypoint.x + (target_waypoint.x - start_waypoint.x) * ratio,
                        y=start_waypoint.y + (target_waypoint.y - start_waypoint.y) * ratio,
                        z=start_waypoint.z + (target_waypoint.z - start_waypoint.z) * ratio,
                        roll_rad=locked_orientation[0],
                        pitch_rad=locked_orientation[1],
                        yaw_rad=locked_orientation[2],
                        velocity_mm_s=target_waypoint.velocity_mm_s,
                        blend_type='FINE',
                    )
                    error_code, fraction = execute_refined_step(
                        current_waypoint,
                        interpolated_waypoint,
                        f'Edge {segment_index} step {step_index}/{step_count}:',
                    )
                    if error_code != SUCCESS_ERROR_CODE:
                        return error_code, (float(step_index - 1) + fraction) / float(step_count)
                    current_waypoint = interpolated_waypoint

                return SUCCESS_ERROR_CODE, 1.0

            return execute_cartesian_waypoint(target_waypoint, f'Edge {segment_index}:')

        for segment_index, waypoint in enumerate(waypoints[1:], start=1):
            self.get_logger().info(f'Executing edge {segment_index}/4...')
            if self.args.settle_time > 0.0:
                time.sleep(self.args.settle_time)
            error_code, planned_fraction = execute_segment(waypoint, segment_index)
            if error_code != SUCCESS_ERROR_CODE:
                self.get_logger().error(
                    f'Square motion failed on edge {segment_index} with error code: {error_code} '
                    f'(planned fraction={planned_fraction:.3f}).'
                )
                return

        self.get_logger().info('Square complete.')

    def _print_waypoints(self, waypoints: list[CartesianWaypoint]) -> None:
        self.get_logger().info('Dry run square waypoints:')
        for index, waypoint in enumerate(waypoints, start=1):
            self.get_logger().info(
                f'  {index}. x={waypoint.x:.4f}, y={waypoint.y:.4f}, z={waypoint.z:.4f}, '
                f'rpy=({math.degrees(waypoint.roll_rad):.2f}, '
                f'{math.degrees(waypoint.pitch_rad):.2f}, '
                f'{math.degrees(waypoint.yaw_rad):.2f})'
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Move the FANUC flange around a square in one plane.')
    parser.add_argument(
        '--file',
        default=str(Path(__file__).with_name('joint_sequence.json')),
        help='Recorded point file used to define the plane.',
    )
    parser.add_argument(
        '--length',
        type=float,
        required=True,
        help='Square side length in meters.',
    )
    parser.add_argument(
        '--point-names',
        nargs=3,
        default=None,
        help='Optional three point names to use instead of the first three non-zero points.',
    )
    parser.add_argument('--planning-group', default='manipulator', help='MoveIt planning group.')
    parser.add_argument('--reference-frame', default='base_link', help='MoveIt reference frame.')
    parser.add_argument('--end-effector-link', default='end_effector', help='End effector link.')
    parser.add_argument('--vel', type=float, default=0.05, help='Velocity scaling.')
    parser.add_argument('--acc', type=float, default=0.05, help='Acceleration scaling.')
    parser.add_argument(
        '--execution-mode',
        choices=['stepwise', 'cartesian'],
        default='stepwise',
        help='Execution style: stepwise is slower but often avoids joint flips.',
    )
    parser.add_argument(
        '--step-size',
        type=float,
        default=0.01,
        help='Step size in meters for stepwise execution mode.',
    )
    parser.add_argument(
        '--max-refine-depth',
        type=int,
        default=3,
        help='Maximum recursive bisections for a failed stepwise segment.',
    )
    parser.add_argument(
        '--min-refine-step',
        type=float,
        default=0.003,
        help='Minimum segment length in meters to keep refining stepwise failures.',
    )
    parser.add_argument('--eef-step', type=float, default=0.01, help='Cartesian path step size.')
    parser.add_argument(
        '--jump-threshold',
        type=float,
        default=0.0,
        help='Cartesian jump threshold. Use 0.0 to disable jump filtering for better continuity on constrained planes.',
    )
    parser.add_argument('--settle-time', type=float, default=0.3, help='Seconds to wait for joint state updates before each Cartesian request.')
    parser.add_argument(
        '--trajectory-start-delay',
        type=float,
        default=0.1,
        help='Delay before the executed trajectory starts.',
    )
    parser.add_argument(
        '--position-tolerance',
        type=float,
        default=0.001,
        help='Cartesian position tolerance in meters.',
    )
    parser.add_argument(
        '--orientation-tolerance',
        type=float,
        default=0.01,
        help='Cartesian orientation tolerance in radians.',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print the computed square waypoints and exit without moving the robot.',
    )
    return parser


def spin_node(node: Node) -> None:
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    rclpy.init()
    node = SquarePlaneDemo(args)
    spinner = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spinner.start()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spinner.join(timeout=2.0)


if __name__ == '__main__':
    main()