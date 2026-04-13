"""Reusable Cartesian motion helpers with FANUC-style FINE and CNT behaviour."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional, Sequence

from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint


DEFAULT_PLANNING_GROUP = 'manipulator'
DEFAULT_END_EFFECTOR_LINK = 'pointer_tcp'
DEFAULT_REFERENCE_FRAME = 'world'
DEFAULT_VELOCITY_SCALE = 0.1
DEFAULT_ACCELERATION_SCALE = 0.1
DEFAULT_NOMINAL_SPEED_MM_S = 500.0
DEFAULT_BLEND_RADIUS_M = 0.1
DEFAULT_EEF_STEP_M = 0.01
DEFAULT_JUMP_THRESHOLD = 0.0
DEFAULT_POSITION_TOLERANCE_M = 0.001
DEFAULT_ORIENTATION_TOLERANCE_RAD = 0.01
DEFAULT_TRAJECTORY_START_DELAY = 0.1
MIN_WAYPOINT_DISTANCE_M = 0.001
MAX_ORIENTATION_STEP_RAD = math.radians(15.0)
SUCCESS_ERROR_CODE = 1
GOAL_REJECTED_ERROR_CODE = 99999


@dataclass(frozen=True)
class CartesianWaypoint:
    """Single Cartesian waypoint with FANUC-style blending metadata."""

    x: float
    y: float
    z: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    velocity_mm_s: float = DEFAULT_NOMINAL_SPEED_MM_S
    blend_type: str = 'FINE'
    blend_radius_m: float = DEFAULT_BLEND_RADIUS_M
    before_move: Optional[Callable[[], None]] = None
    after_move: Optional[Callable[[], None]] = None

    def normalized_blend_type(self) -> str:
        """Return uppercase blend type with `FINE` fallback."""
        blend = str(self.blend_type).strip().upper()
        return blend if blend in {'FINE', 'CNT'} else 'FINE'

    def to_pose(self) -> Pose:
        """Convert waypoint to `Pose`."""
        qx, qy, qz, qw = rpy_to_quaternion(
            self.roll_rad,
            self.pitch_rad,
            self.yaw_rad,
        )
        pose = Pose()
        pose.position = Point(x=float(self.x), y=float(self.y), z=float(self.z))
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return pose


def rpy_to_quaternion(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> tuple[float, float, float, float]:
    """Convert roll/pitch/yaw in radians to quaternion `(x, y, z, w)`."""
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_to_rpy(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[float, float, float]:
    """Convert quaternion `(x, y, z, w)` to roll/pitch/yaw in radians."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def slerp_quaternion(
    q1: Sequence[float],
    q2: Sequence[float],
    t: float,
) -> tuple[float, float, float, float]:
    """Spherical linear interpolation between two quaternions."""
    t = max(0.0, min(1.0, float(t)))
    q1x, q1y, q1z, q1w = _normalize_quaternion(tuple(q1))
    q2x, q2y, q2z, q2w = _normalize_quaternion(tuple(q2))

    dot = q1x * q2x + q1y * q2y + q1z * q2z + q1w * q2w
    if dot < 0.0:
        q2x, q2y, q2z, q2w = (-q2x, -q2y, -q2z, -q2w)
        dot = -dot

    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return _normalize_quaternion(
            (
                q1x + t * (q2x - q1x),
                q1y + t * (q2y - q1y),
                q1z + t * (q2z - q1z),
                q1w + t * (q2w - q1w),
            )
        )

    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * t
    sin_theta = math.sin(theta)

    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return _normalize_quaternion(
        (
            s0 * q1x + s1 * q2x,
            s0 * q1y + s1 * q2y,
            s0 * q1z + s1 * q2z,
            s0 * q1w + s1 * q2w,
        )
    )


def validate_waypoint_spacing(
    waypoints: Sequence[CartesianWaypoint],
    *,
    min_distance_m: float = MIN_WAYPOINT_DISTANCE_M,
    max_orientation_delta_rad: float = math.radians(90.0),
) -> tuple[bool, str]:
    """Reject degenerate or abrupt waypoint jumps."""
    if len(waypoints) < 2:
        return True, ''

    for index in range(len(waypoints) - 1):
        current_waypoint = waypoints[index]
        next_waypoint = waypoints[index + 1]
        position_distance = math.dist(
            (current_waypoint.x, current_waypoint.y, current_waypoint.z),
            (next_waypoint.x, next_waypoint.y, next_waypoint.z),
        )
        if position_distance < min_distance_m:
            return False, (
                f'Waypoint {index + 1} and {index + 2} are only '
                f'{position_distance * 1000.0:.3f} mm apart.'
            )

        q1 = rpy_to_quaternion(
            current_waypoint.roll_rad,
            current_waypoint.pitch_rad,
            current_waypoint.yaw_rad,
        )
        q2 = rpy_to_quaternion(
            next_waypoint.roll_rad,
            next_waypoint.pitch_rad,
            next_waypoint.yaw_rad,
        )
        orientation_delta = quaternion_angle_distance(q1, q2)
        if orientation_delta > max_orientation_delta_rad:
            return False, (
                f'Waypoint {index + 1} and {index + 2} rotate '
                f'{math.degrees(orientation_delta):.1f} deg, which is too abrupt.'
            )

    return True, ''


def quaternion_angle_distance(
    q1: Sequence[float],
    q2: Sequence[float],
) -> float:
    """Return shortest angular distance between two orientations."""
    q1x, q1y, q1z, q1w = _normalize_quaternion(tuple(q1))
    q2x, q2y, q2z, q2w = _normalize_quaternion(tuple(q2))
    dot = abs(q1x * q2x + q1y * q2y + q1z * q2z + q1w * q2w)
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def interpolate_cartesian_segment(
    start_waypoint: CartesianWaypoint,
    end_waypoint: CartesianWaypoint,
    *,
    max_position_step_m: float = DEFAULT_EEF_STEP_M,
    max_orientation_step_rad: float = MAX_ORIENTATION_STEP_RAD,
) -> list[Pose]:
    """Insert intermediate poses so CNT paths stay smooth and predictable."""
    start_q = rpy_to_quaternion(
        start_waypoint.roll_rad,
        start_waypoint.pitch_rad,
        start_waypoint.yaw_rad,
    )
    end_q = rpy_to_quaternion(
        end_waypoint.roll_rad,
        end_waypoint.pitch_rad,
        end_waypoint.yaw_rad,
    )
    position_distance = math.dist(
        (start_waypoint.x, start_waypoint.y, start_waypoint.z),
        (end_waypoint.x, end_waypoint.y, end_waypoint.z),
    )
    orientation_distance = quaternion_angle_distance(start_q, end_q)

    step_count = max(
        1,
        math.ceil(position_distance / max(max_position_step_m, 1e-6)),
        math.ceil(orientation_distance / max(max_orientation_step_rad, 1e-6)),
    )

    poses = []
    for step_index in range(step_count + 1):
        t = step_index / step_count
        pose = Pose()
        pose.position = Point(
            x=start_waypoint.x + (end_waypoint.x - start_waypoint.x) * t,
            y=start_waypoint.y + (end_waypoint.y - start_waypoint.y) * t,
            z=start_waypoint.z + (end_waypoint.z - start_waypoint.z) * t,
        )
        qx, qy, qz, qw = slerp_quaternion(start_q, end_q, t)
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        poses.append(pose)
    return poses


def densify_cartesian_path(
    waypoints: Sequence[CartesianWaypoint],
    *,
    max_position_step_m: float = DEFAULT_EEF_STEP_M,
    max_orientation_step_rad: float = MAX_ORIENTATION_STEP_RAD,
) -> list[Pose]:
    """Return dense poses for CNT execution so the solver avoids odd shortcuts."""
    if not waypoints:
        return []
    if len(waypoints) == 1:
        return [waypoints[0].to_pose()]

    poses = [waypoints[0].to_pose()]
    for index in range(len(waypoints) - 1):
        segment = interpolate_cartesian_segment(
            waypoints[index],
            waypoints[index + 1],
            max_position_step_m=max_position_step_m,
            max_orientation_step_rad=max_orientation_step_rad,
        )
        poses.extend(segment[1:])
    return poses


def build_cartesian_constraints(
    target_pose: Pose,
    *,
    reference_frame: str = DEFAULT_REFERENCE_FRAME,
    end_effector_link: str = DEFAULT_END_EFFECTOR_LINK,
    position_tolerance: float = DEFAULT_POSITION_TOLERANCE_M,
    orientation_tolerance: float = DEFAULT_ORIENTATION_TOLERANCE_RAD,
    weight: float = 1.0,
) -> Constraints:
    """Build MoveIt pose constraints for one FINE pose target."""
    constraints = Constraints()

    header = Header()
    header.frame_id = reference_frame

    position_constraint = PositionConstraint()
    position_constraint.header = header
    position_constraint.link_name = end_effector_link
    position_constraint.target_point_offset.x = 0.0
    position_constraint.target_point_offset.y = 0.0
    position_constraint.target_point_offset.z = 0.0
    position_constraint.weight = weight

    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [position_tolerance]

    bounding_volume = BoundingVolume()
    bounding_volume.primitives.append(sphere)

    center_pose = Pose()
    center_pose.position = Point(
        x=target_pose.position.x,
        y=target_pose.position.y,
        z=target_pose.position.z,
    )
    center_pose.orientation = Quaternion(w=1.0)
    bounding_volume.primitive_poses.append(center_pose)
    position_constraint.constraint_region = bounding_volume

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header = header
    orientation_constraint.link_name = end_effector_link
    orientation_constraint.orientation = target_pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance
    orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance
    orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance
    orientation_constraint.weight = weight

    constraints.position_constraints.append(position_constraint)
    constraints.orientation_constraints.append(orientation_constraint)
    return constraints


def build_move_group_goal(
    waypoint: CartesianWaypoint,
    *,
    planning_group: str = DEFAULT_PLANNING_GROUP,
    end_effector_link: str = DEFAULT_END_EFFECTOR_LINK,
    reference_frame: str = DEFAULT_REFERENCE_FRAME,
    velocity_scale: float = DEFAULT_VELOCITY_SCALE,
    acceleration_scale: float = DEFAULT_ACCELERATION_SCALE,
    position_tolerance: float = DEFAULT_POSITION_TOLERANCE_M,
    orientation_tolerance: float = DEFAULT_ORIENTATION_TOLERANCE_RAD,
) -> MoveGroup.Goal:
    """Build a `MoveGroup.Goal` for one Cartesian FINE waypoint."""
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = planning_group
    goal.request.max_velocity_scaling_factor = float(velocity_scale)
    goal.request.max_acceleration_scaling_factor = float(acceleration_scale)
    goal.request.goal_constraints.append(
        build_cartesian_constraints(
            waypoint.to_pose(),
            reference_frame=reference_frame,
            end_effector_link=end_effector_link,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
        )
    )
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = False
    return goal


def build_cartesian_path_request(
    waypoints: Sequence[Pose],
    *,
    current_joint_state: Optional[JointState],
    planning_group: str,
    end_effector_link: str,
    reference_frame: str,
    eef_step: float,
    jump_threshold: float,
) -> GetCartesianPath.Request:
    """Build the `GetCartesianPath` request used for CNT moves."""
    request = GetCartesianPath.Request()
    request.header = Header()
    request.header.frame_id = reference_frame
    request.start_state = build_start_state(current_joint_state)
    request.group_name = planning_group
    request.link_name = end_effector_link
    request.max_step = eef_step
    request.jump_threshold = jump_threshold
    request.avoid_collisions = False
    request.waypoints = list(waypoints)
    return request


def build_start_state(current_joint_state: Optional[JointState]) -> RobotState:
    """Build a MoveIt start state from the latest `/joint_states` message."""
    start_state = RobotState()
    start_state.is_diff = True
    if current_joint_state is None:
        return start_state

    start_state.joint_state.name = list(current_joint_state.name)
    start_state.joint_state.position = list(current_joint_state.position)
    return start_state


def apply_velocity_acceleration_scaling(
    trajectory,
    *,
    velocity_scale: float,
    acceleration_scale: float,
) -> None:
    """Slow down or speed up a trajectory while preserving its shape."""
    points = trajectory.joint_trajectory.points
    if not points:
        return

    velocity_scale = max(0.001, min(1.0, float(velocity_scale)))
    acceleration_scale = max(0.001, min(1.0, float(acceleration_scale)))

    for point in points:
        original_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        scaled_time = original_time / velocity_scale
        point.time_from_start.sec = int(scaled_time)
        point.time_from_start.nanosec = int((scaled_time - int(scaled_time)) * 1e9)
        if point.velocities:
            point.velocities = [velocity_scale * value for value in point.velocities]
        if point.accelerations:
            point.accelerations = [acceleration_scale * value for value in point.accelerations]


def prepend_current_state_point(
    trajectory,
    current_joint_state: Optional[JointState],
    *,
    start_delay: float = DEFAULT_TRAJECTORY_START_DELAY,
) -> None:
    """Insert a zero-velocity start point so execution starts smoothly."""
    if current_joint_state is None:
        return

    joint_names = list(trajectory.joint_trajectory.joint_names)
    if not joint_names:
        return

    current_positions = dict(zip(current_joint_state.name, current_joint_state.position))
    if any(name not in current_positions for name in joint_names):
        return

    shift = max(0.0, float(start_delay))
    for point in trajectory.joint_trajectory.points:
        original_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        shifted_time = original_time + shift
        point.time_from_start.sec = int(shifted_time)
        point.time_from_start.nanosec = int((shifted_time - int(shifted_time)) * 1e9)

    start_point = JointTrajectoryPoint()
    start_point.positions = [current_positions[name] for name in joint_names]
    start_point.velocities = [0.0] * len(joint_names)
    start_point.accelerations = [0.0] * len(joint_names)
    start_point.time_from_start.sec = 0
    start_point.time_from_start.nanosec = 0
    trajectory.joint_trajectory.points.insert(0, start_point)


class CartesianMotionClient:
    """Reusable ROS client for Cartesian FINE and CNT motion."""

    def __init__(
        self,
        node: Node,
        *,
        planning_group: str = DEFAULT_PLANNING_GROUP,
        end_effector_link: str = DEFAULT_END_EFFECTOR_LINK,
        reference_frame: str = DEFAULT_REFERENCE_FRAME,
        vel: float = DEFAULT_VELOCITY_SCALE,
        acc: float = DEFAULT_ACCELERATION_SCALE,
        nominal_speed_mm_s: float = DEFAULT_NOMINAL_SPEED_MM_S,
        eef_step: float = DEFAULT_EEF_STEP_M,
        jump_threshold: float = DEFAULT_JUMP_THRESHOLD,
        trajectory_start_delay: float = DEFAULT_TRAJECTORY_START_DELAY,
        position_tolerance: float = DEFAULT_POSITION_TOLERANCE_M,
        orientation_tolerance: float = DEFAULT_ORIENTATION_TOLERANCE_RAD,
        move_action_name: str = '/move_action',
        cartesian_service_name: str = '/compute_cartesian_path',
        execute_action_name: str = '/execute_trajectory',
        joint_state_topic: str = '/joint_states',
    ):
        self.node = node
        self.planning_group = planning_group
        self.end_effector_link = end_effector_link
        self.reference_frame = reference_frame
        self.vel = float(vel)
        self.acc = float(acc)
        self.nominal_speed_mm_s = float(nominal_speed_mm_s)
        self.eef_step = float(eef_step)
        self.jump_threshold = float(jump_threshold)
        self.trajectory_start_delay = float(trajectory_start_delay)
        self.position_tolerance = float(position_tolerance)
        self.orientation_tolerance = float(orientation_tolerance)

        self.move_action_client = ActionClient(node, MoveGroup, move_action_name)
        self.execute_action_client = ActionClient(node, ExecuteTrajectory, execute_action_name)
        self.cartesian_path_client = node.create_client(
            GetCartesianPath,
            cartesian_service_name,
        )
        self.current_joint_state: Optional[JointState] = None
        self.node.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            10,
        )

    def wait_for_move_action(self, timeout_sec: Optional[float] = None) -> bool:
        """Wait for `/move_action`."""
        if timeout_sec is None:
            self.move_action_client.wait_for_server()
            return True
        return self.move_action_client.wait_for_server(timeout_sec=timeout_sec)

    def wait_for_cartesian_path_service(self, timeout_sec: Optional[float] = None) -> bool:
        """Wait for `/compute_cartesian_path`."""
        if timeout_sec is None:
            return self.cartesian_path_client.wait_for_service()
        return self.cartesian_path_client.wait_for_service(timeout_sec=timeout_sec)

    def wait_for_execute_action(self, timeout_sec: Optional[float] = None) -> bool:
        """Wait for `/execute_trajectory`."""
        if timeout_sec is None:
            self.execute_action_client.wait_for_server()
            return True
        return self.execute_action_client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal(
        self,
        waypoint: CartesianWaypoint,
        *,
        goal_response_callback: Optional[Callable[[object], None]] = None,
        result_callback: Optional[Callable[[int, object], None]] = None,
    ):
        """Send one FINE Cartesian pose goal asynchronously."""
        if waypoint.before_move is not None:
            waypoint.before_move()

        velocity_scale = self._waypoint_velocity_scale(waypoint)
        goal = build_move_group_goal(
            waypoint,
            planning_group=self.planning_group,
            end_effector_link=self.end_effector_link,
            reference_frame=self.reference_frame,
            velocity_scale=velocity_scale,
            acceleration_scale=self.acc,
            position_tolerance=self.position_tolerance,
            orientation_tolerance=self.orientation_tolerance,
        )
        future = self.move_action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done_future: self._handle_move_goal_response(
                done_future,
                waypoint.after_move,
                goal_response_callback,
                result_callback,
            )
        )
        return future

    def send_path(
        self,
        waypoints: Sequence[CartesianWaypoint],
        *,
        goal_response_callback: Optional[Callable[[object], None]] = None,
        result_callback: Optional[Callable[[int, object], None]] = None,
    ):
        """Send one CNT Cartesian path asynchronously."""
        if not waypoints:
            if result_callback is not None:
                result_callback(GOAL_REJECTED_ERROR_CODE, None)
            return None

        is_valid, reason = validate_waypoint_spacing(waypoints)
        if not is_valid:
            self.node.get_logger().error(f'Invalid CNT sequence: {reason}')
            if result_callback is not None:
                result_callback(GOAL_REJECTED_ERROR_CODE, None)
            return None

        if waypoints[0].before_move is not None:
            waypoints[0].before_move()

        dense_poses = densify_cartesian_path(
            waypoints,
            max_position_step_m=self.eef_step,
            max_orientation_step_rad=MAX_ORIENTATION_STEP_RAD,
        )
        request = build_cartesian_path_request(
            dense_poses,
            current_joint_state=self.current_joint_state,
            planning_group=self.planning_group,
            end_effector_link=self.end_effector_link,
            reference_frame=self.reference_frame,
            eef_step=self.eef_step,
            jump_threshold=self.jump_threshold,
        )
        future = self.cartesian_path_client.call_async(request)
        future.add_done_callback(
            lambda done_future: self._handle_cartesian_path_response(
                done_future,
                waypoints[-1].after_move,
                goal_response_callback,
                result_callback,
                velocity_scale=min(
                    self._waypoint_velocity_scale(waypoint) for waypoint in waypoints
                ),
            )
        )
        return future

    def _joint_state_callback(self, message: JointState) -> None:
        self.current_joint_state = message

    def _handle_move_goal_response(
        self,
        future,
        after_move: Optional[Callable[[], None]],
        goal_response_callback: Optional[Callable[[object], None]],
        result_callback: Optional[Callable[[int, object], None]],
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.node.get_logger().error(f'MoveIt goal send failed: {exc}')
            if result_callback is not None:
                result_callback(GOAL_REJECTED_ERROR_CODE, None)
            return

        if goal_response_callback is not None:
            goal_response_callback(goal_handle)

        if not goal_handle.accepted:
            if result_callback is not None:
                result_callback(GOAL_REJECTED_ERROR_CODE, None)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future: self._handle_action_result(
                done_future,
                after_move,
                result_callback,
            )
        )

    def _handle_cartesian_path_response(
        self,
        future,
        after_move: Optional[Callable[[], None]],
        goal_response_callback: Optional[Callable[[object], None]],
        result_callback: Optional[Callable[[int, object], None]],
        *,
        velocity_scale: float,
    ) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.node.get_logger().error(f'Cartesian path request failed: {exc}')
            if result_callback is not None:
                result_callback(GOAL_REJECTED_ERROR_CODE, None)
            return

        if response.error_code.val != SUCCESS_ERROR_CODE or response.fraction < 0.999:
            self.node.get_logger().error(
                'Cartesian path could not be planned completely '
                f'(fraction={response.fraction:.3f}, code={response.error_code.val}).'
            )
            if result_callback is not None:
                result_callback(response.error_code.val, response)
            return

        trajectory = response.solution
        apply_velocity_acceleration_scaling(
            trajectory,
            velocity_scale=velocity_scale,
            acceleration_scale=self.acc,
        )
        prepend_current_state_point(
            trajectory,
            self.current_joint_state,
            start_delay=self.trajectory_start_delay,
        )

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        execute_future = self.execute_action_client.send_goal_async(goal)
        execute_future.add_done_callback(
            lambda done_future: self._handle_move_goal_response(
                done_future,
                after_move,
                goal_response_callback,
                result_callback,
            )
        )

    def _handle_action_result(
        self,
        future,
        after_move: Optional[Callable[[], None]],
        result_callback: Optional[Callable[[int, object], None]],
    ) -> None:
        wrapped_result = future.result()
        result = wrapped_result.result
        error_code = result.error_code.val
        if error_code == SUCCESS_ERROR_CODE and after_move is not None:
            after_move()
        if result_callback is not None:
            result_callback(error_code, result)

    def _waypoint_velocity_scale(self, waypoint: CartesianWaypoint) -> float:
        ratio = float(waypoint.velocity_mm_s) / max(self.nominal_speed_mm_s, 1e-6)
        return max(0.001, min(1.0, self.vel * ratio))


def _normalize_quaternion(
    quaternion: Sequence[float],
) -> tuple[float, float, float, float]:
    qx, qy, qz, qw = [float(value) for value in quaternion]
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm
