"""
move_linear.py
==============
True linear Cartesian square motion for FANUC CRX-10iA/L.

What it does:
    - Uses /compute_cartesian_path to guarantee straight Cartesian edges
    - Builds one closed square trajectory:
      corner_1 -> corner_2 -> corner_3 -> corner_4 -> corner_1
    - Executes the trajectory using /execute_trajectory
    - Repeats forever with delay_between_squares

Usage:
    ros2 launch fanuc_tools move_linear.launch.py use_mock:=true use_rviz:=false
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


def rpy_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """
    Convert Roll, Pitch, Yaw (in degrees) to a quaternion.

    YAML uses degrees because they're human readable.
    MoveIt requires quaternions internally.
    This function bridges the two.

    Args:
        roll_deg:  rotation around X axis in degrees
        pitch_deg: rotation around Y axis in degrees
        yaw_deg:   rotation around Z axis in degrees

    Returns:
        Quaternion message (x, y, z, w)
    """
    # Convert degrees to radians
    roll  = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw   = math.radians(yaw_deg)

    # Standard RPY to quaternion formula
    cy = math.cos(yaw   * 0.5)
    sy = math.sin(yaw   * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll  * 0.5)
    sr = math.sin(roll  * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q


class MoveLinearNode(Node):
    """Compute and execute a closed Cartesian square as one linear trajectory."""

    def __init__(self):
        super().__init__('move_linear_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('end_effector_link', 'pointer_tcp')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('vel', 0.1)
        self.declare_parameter('acc', 0.1)
        self.declare_parameter('eef_step', 0.01)
        self.declare_parameter('jump_threshold', 0.0)
        self.declare_parameter('startup_delay', 5.0)
        self.declare_parameter('align_retry_delay', 3.0)
        self.declare_parameter('trajectory_start_delay', 2.0)
        self.declare_parameter('delay_between_squares', 2.0)

        # Corner 1
        self.declare_parameter('corner_1.x', 0.5)
        self.declare_parameter('corner_1.y', 0.1)
        self.declare_parameter('corner_1.z', 0.5)
        self.declare_parameter('corner_1.roll', 180.0)
        self.declare_parameter('corner_1.pitch', 0.0)
        self.declare_parameter('corner_1.yaw', 0.0)

        # Corner 2
        self.declare_parameter('corner_2.x', 0.7)
        self.declare_parameter('corner_2.y', 0.1)
        self.declare_parameter('corner_2.z', 0.5)
        self.declare_parameter('corner_2.roll', 180.0)
        self.declare_parameter('corner_2.pitch', 0.0)
        self.declare_parameter('corner_2.yaw', 0.0)

        # Corner 3
        self.declare_parameter('corner_3.x', 0.7)
        self.declare_parameter('corner_3.y', -0.1)
        self.declare_parameter('corner_3.z', 0.5)
        self.declare_parameter('corner_3.roll', 180.0)
        self.declare_parameter('corner_3.pitch', 0.0)
        self.declare_parameter('corner_3.yaw', 0.0)

        # Corner 4
        self.declare_parameter('corner_4.x', 0.5)
        self.declare_parameter('corner_4.y', -0.1)
        self.declare_parameter('corner_4.z', 0.5)
        self.declare_parameter('corner_4.roll', 180.0)
        self.declare_parameter('corner_4.pitch', 0.0)
        self.declare_parameter('corner_4.yaw', 0.0)

        # ── Read parameters ──────────────────────────────────────────────────
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.vel = float(self.get_parameter('vel').value)
        self.acc = float(self.get_parameter('acc').value)
        self.eef_step = float(self.get_parameter('eef_step').value)
        self.jump_threshold = float(self.get_parameter('jump_threshold').value)
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.align_retry_delay = float(self.get_parameter('align_retry_delay').value)
        self.trajectory_start_delay = float(
            self.get_parameter('trajectory_start_delay').value
        )
        self.delay_between_squares = float(self.get_parameter('delay_between_squares').value)

        self.corners = [
            self._load_corner('corner_1', 1),
            self._load_corner('corner_2', 2),
            self._load_corner('corner_3', 3),
            self._load_corner('corner_4', 4),
        ]

        self.get_logger().info(f'Planning group         : {self.planning_group}')
        self.get_logger().info(f'End effector link      : {self.end_effector_link}')
        self.get_logger().info(f'Reference frame        : {self.reference_frame}')
        self.get_logger().info(f'Velocity scale         : {self.vel}')
        self.get_logger().info(f'Accel scale            : {self.acc}')
        self.get_logger().info(f'eef_step               : {self.eef_step}')
        self.get_logger().info(f'jump_threshold         : {self.jump_threshold}')
        self.get_logger().info(f'Startup delay          : {self.startup_delay}s')
        self.get_logger().info(f'Align retry delay      : {self.align_retry_delay}s')
        self.get_logger().info(f'Trajectory start delay : {self.trajectory_start_delay}s')
        self.get_logger().info(f'Delay between squares  : {self.delay_between_squares}s')

        for corner in self.corners:
            pose = corner['pose']
            self.get_logger().info(
                f'{corner["name"]}: ({pose.position.x:.3f}, '
                f'{pose.position.y:.3f}, {pose.position.z:.3f})'
            )

        # ── Interfaces ───────────────────────────────────────────────────────
        self.current_joint_state = None
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

        self.cartesian_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path',
        )

        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory',
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Loop state ───────────────────────────────────────────────────────
        self.iteration = 0
        self.motion_in_progress = False
        self.initial_alignment_done = False
        self.start_time = time.monotonic()
        self.next_attempt_time = self.start_time + self.startup_delay
        self.startup_wait_logged = False

        self.create_timer(1.0, self.loop_tick)

    def _load_corner(self, base_name, corner_index):
        roll_deg = 180.0
        pitch_deg = 0.0
        yaw_deg = 0.0

        pose = Pose(
            position=Point(
                x=float(self.get_parameter(f'{base_name}.x').value),
                y=float(self.get_parameter(f'{base_name}.y').value),
                z=float(self.get_parameter(f'{base_name}.z').value),
            ),
            orientation=rpy_to_quaternion(roll_deg, pitch_deg, yaw_deg),
        )

        return {
            'name': f'Corner {corner_index}',
            'pose': pose,
        }

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def build_start_state(self):
        start_state = RobotState()
        start_state.is_diff = True
        if self.current_joint_state is None:
            return start_state

        start_state.joint_state.name = list(self.current_joint_state.name)
        start_state.joint_state.position = list(self.current_joint_state.position)
        return start_state

    def get_current_tcp_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.end_effector_link,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Unable to read TF {self.reference_frame} -> {self.end_effector_link}: {exc}'
            )
            return None

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def loop_tick(self):
        if self.motion_in_progress:
            return

        now = time.monotonic()
        if now < self.next_attempt_time:
            if not self.startup_wait_logged:
                remaining = max(0.0, self.next_attempt_time - now)
                self.get_logger().info(
                    f'Waiting {remaining:.1f}s before next Cartesian planning attempt...'
                )
                self.startup_wait_logged = True
            return

        self.startup_wait_logged = False

        self.motion_in_progress = True
        self.iteration += 1

        self.get_logger().info('─' * 40)

        if not self.initial_alignment_done:
            self.get_logger().info(
                f'Iteration {self.iteration} — Aligning to Corner 1 before square loop'
            )
            self.plan_initial_alignment()
            return

        self.get_logger().info(f'Iteration {self.iteration} — Planning linear square path')
        for corner in self.corners:
            pose = corner['pose']
            self.get_logger().info(
                f'Heading to {corner["name"]} '
                f'({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
            )
        self.get_logger().info(
            f'Heading to Corner 1 '
            f'({self.corners[0]["pose"].position.x:.3f}, '
            f'{self.corners[0]["pose"].position.y:.3f}, '
            f'{self.corners[0]["pose"].position.z:.3f})'
        )
        self.plan_square_loop()

    def request_cartesian_path(self, waypoints, mode):
        if not self.cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/compute_cartesian_path service not available yet.')
            self.next_attempt_time = time.monotonic() + self.align_retry_delay
            self.motion_in_progress = False
            return

        request = GetCartesianPath.Request()
        request.header = Header()
        request.header.frame_id = self.reference_frame
        request.start_state = self.build_start_state()
        request.group_name = self.planning_group
        request.link_name = self.end_effector_link
        request.max_step = self.eef_step
        request.jump_threshold = self.jump_threshold
        request.avoid_collisions = False
        request.waypoints = waypoints

        future = self.cartesian_client.call_async(request)
        future.add_done_callback(lambda f: self.cartesian_path_callback(f, mode))

    def plan_initial_alignment(self):
        current_pose = self.get_current_tcp_pose()
        if current_pose is None:
            self.motion_in_progress = False
            return

        desired_orientation = rpy_to_quaternion(180.0, 0.0, 0.0)
        corner_1 = self.corners[0]['pose']

        approach_z = max(corner_1.position.z + 0.15, current_pose.position.z)

        approach_pose = Pose()
        approach_pose.position = Point(
            x=corner_1.position.x,
            y=corner_1.position.y,
            z=approach_z,
        )
        approach_pose.orientation = current_pose.orientation

        move_to_corner_1_pose = Pose()
        move_to_corner_1_pose.position = Point(
            x=corner_1.position.x,
            y=corner_1.position.y,
            z=corner_1.position.z,
        )
        move_to_corner_1_pose.orientation = current_pose.orientation

        orient_at_corner_1_pose = Pose()
        orient_at_corner_1_pose.position = Point(
            x=corner_1.position.x,
            y=corner_1.position.y,
            z=corner_1.position.z,
        )
        orient_at_corner_1_pose.orientation = desired_orientation

        self.request_cartesian_path(
            [approach_pose, move_to_corner_1_pose, orient_at_corner_1_pose],
            mode='align',
        )

    def plan_square_loop(self):
        desired_orientation = rpy_to_quaternion(180.0, 0.0, 0.0)
        waypoints = []
        for corner in self.corners:
            pose = Pose()
            pose.position = Point(
                x=corner['pose'].position.x,
                y=corner['pose'].position.y,
                z=corner['pose'].position.z,
            )
            pose.orientation = desired_orientation
            waypoints.append(pose)

        first = Pose()
        first.position = Point(
            x=waypoints[0].position.x,
            y=waypoints[0].position.y,
            z=waypoints[0].position.z,
        )
        first.orientation = desired_orientation
        waypoints.append(first)

        self.request_cartesian_path(waypoints, mode='square')

    def cartesian_path_callback(self, future, mode):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Cartesian path service call failed: {exc}')
            self.next_attempt_time = time.monotonic() + self.align_retry_delay
            self.motion_in_progress = False
            return

        if response.error_code.val != 1:
            self.get_logger().error(
                f'Cartesian planning failed (error code: {response.error_code.val})'
            )
            self.next_attempt_time = time.monotonic() + self.align_retry_delay
            self.motion_in_progress = False
            return

        self.get_logger().info(f'Cartesian path fraction: {response.fraction:.6f}')

        if response.fraction < 1.0:
            if mode == 'align':
                self.get_logger().warn(
                    f'Initial alignment path incomplete (fraction={response.fraction:.6f}). '
                    f'Retrying in {self.align_retry_delay:.1f}s.'
                )
                self.next_attempt_time = time.monotonic() + self.align_retry_delay
            else:
                self.get_logger().warn(
                    f'Path is not fully planned (fraction={response.fraction:.6f}). '
                    f'Expected 1.0. Motion not executed.'
                )
                self.next_attempt_time = time.monotonic() + self.delay_between_squares
            self.motion_in_progress = False
            return

        trajectory = response.solution
        self.apply_velocity_acceleration_scaling(trajectory)
        self.prepend_current_state_point(trajectory)
        self.execute_trajectory(trajectory, mode)

    def apply_velocity_acceleration_scaling(self, trajectory):
        points = trajectory.joint_trajectory.points
        if not points:
            return

        velocity_scale = max(0.001, min(1.0, self.vel))
        acceleration_scale = max(0.001, min(1.0, self.acc))

        for point in points:
            original_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            scaled_time = original_time / velocity_scale
            point.time_from_start.sec = int(scaled_time)
            point.time_from_start.nanosec = int((scaled_time - int(scaled_time)) * 1e9)

            if point.velocities:
                point.velocities = [velocity_scale * value for value in point.velocities]

            if point.accelerations:
                point.accelerations = [acceleration_scale * value for value in point.accelerations]

    def prepend_current_state_point(self, trajectory):
        if self.current_joint_state is None:
            return

        joint_names = list(trajectory.joint_trajectory.joint_names)
        if not joint_names:
            return

        current_positions = dict(
            zip(self.current_joint_state.name, self.current_joint_state.position)
        )

        if any(name not in current_positions for name in joint_names):
            self.get_logger().warn(
                'Current joint state does not contain all trajectory joints; '
                'skipping trajectory start padding.'
            )
            return

        shift = max(0.1, self.trajectory_start_delay)
        for point in trajectory.joint_trajectory.points:
            original_time = (
                point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            )
            shifted_time = original_time + shift
            point.time_from_start.sec = int(shifted_time)
            point.time_from_start.nanosec = int(
                (shifted_time - int(shifted_time)) * 1e9
            )

        start_point = JointTrajectoryPoint()
        start_point.positions = [current_positions[name] for name in joint_names]
        start_point.velocities = [0.0] * len(joint_names)
        start_point.accelerations = [0.0] * len(joint_names)
        start_point.time_from_start.sec = 0
        start_point.time_from_start.nanosec = 0

        trajectory.joint_trajectory.points.insert(0, start_point)

    def execute_trajectory(self, trajectory, mode):
        if not self.execute_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('/execute_trajectory action server not available yet.')
            self.next_attempt_time = time.monotonic() + self.align_retry_delay
            self.motion_in_progress = False
            return

        if mode == 'align':
            self.get_logger().info('Executing initial alignment trajectory to Corner 1...')
        else:
            self.get_logger().info('Executing full linear square trajectory...')

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        future = self.execute_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.goal_response_callback(f, mode))

    def goal_response_callback(self, future, mode):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('ExecuteTrajectory goal rejected.')
            self.next_attempt_time = time.monotonic() + self.align_retry_delay
            self.motion_in_progress = False
            return

        self.get_logger().info('ExecuteTrajectory goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.result_callback(f, mode))

    def result_callback(self, future, mode):
        result = future.result().result
        error_code = result.error_code.val

        if error_code == 1:
            if mode == 'align':
                self.get_logger().info('Initial alignment to Corner 1 complete. ✓')
                self.initial_alignment_done = True
            else:
                self.get_logger().info('Square trajectory complete. ✓')
        else:
            self.get_logger().error(f'Linear execution failed (error code: {error_code})')
            self.next_attempt_time = time.monotonic() + self.align_retry_delay
            self.motion_in_progress = False
            return

        if mode == 'align':
            self.next_attempt_time = time.monotonic()
            self.motion_in_progress = False
            return

        self.get_logger().info(
            f'Waiting {self.delay_between_squares}s before next square...'
        )
        time.sleep(self.delay_between_squares)
        self.next_attempt_time = time.monotonic()
        self.motion_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    node = MoveLinearNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Linear square loop stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
