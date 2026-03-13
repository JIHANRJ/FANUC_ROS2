"""
move_cartesian.py
=================
Example: Move the FANUC CRX-10iA/L to a target Cartesian pose using MoveIt2.

What it does:
    Reads two target poses (A and B) defined as X, Y, Z + Roll, Pitch, Yaw
    from parameters, then continuously moves between them with a configurable
    delay — same loop pattern as move_joint.py.

When to use it:
    Use Cartesian motion when you care about WHERE the TCP ends up in space
    (e.g. above a part, at a pick point) rather than what the joints are doing.
    Unlike joint motion, the path is not guaranteed to be a straight line.
    Use move_linear.py if you need a straight line path.

Key concepts:
    - Pose goal: X/Y/Z position + quaternion orientation
    - RPY to quaternion: Roll/Pitch/Yaw angles converted internally
    - /move_action: same MoveIt action server used in move_joint.py
    - Frame: all poses are in the 'world' frame by default

Coordinate system (FANUC CRX-10iA/L):
    X = forward
    Y = left
    Z = up
    Units: metres for position, degrees for RPY in YAML (converted internally)

Quick run:
    source /opt/ros/humble/setup.bash && source ~/ws_fanuc/install/setup.bash
    ros2 launch fanuc_tools move_cartesian.launch.py use_mock:=true use_rviz:=false
    ros2 run fanuc_tools speed_scaling

Run alongside speed_scaling.py to control speed during motion:
    Terminal 1: ros2 launch fanuc_moveit_config fanuc_moveit.launch.py
                    robot_model:=crx10ia_l use_mock:=true
    Terminal 2: ros2 run fanuc_tools move_cartesian
    Terminal 3: ros2 run fanuc_tools speed_scaling

Usage:
    ros2 launch fanuc_tools move_cartesian.launch.py use_mock:=true
"""

import math
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    MotionPlanRequest,
    PlanningOptions,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


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


class MoveCartesianNode(Node):
    """
    Moves the FANUC CRX-10iA/L back and forth between two Cartesian
    poses loaded from parameters.
    """

    def __init__(self):
        super().__init__('move_cartesian_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('end_effector_link', 'end_effector')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('vel', 0.1)
        self.declare_parameter('acc', 0.1)
        self.declare_parameter('delay_between_moves', 2.0)
        self.declare_parameter('position_tolerance', 0.001)      # metres
        self.declare_parameter('orientation_tolerance', 0.01)    # radians

        # Position A
        self.declare_parameter('position_a.x',     0.0)
        self.declare_parameter('position_a.y',     0.0)
        self.declare_parameter('position_a.z',     0.0)
        self.declare_parameter('position_a.roll',  0.0)
        self.declare_parameter('position_a.pitch', 0.0)
        self.declare_parameter('position_a.yaw',   0.0)

        # Position B
        self.declare_parameter('position_b.x',     0.0)
        self.declare_parameter('position_b.y',     0.0)
        self.declare_parameter('position_b.z',     0.0)
        self.declare_parameter('position_b.roll',  0.0)
        self.declare_parameter('position_b.pitch', 0.0)
        self.declare_parameter('position_b.yaw',   0.0)

        # ── Read parameters ──────────────────────────────────────────────────
        self.planning_group        = self.get_parameter('planning_group').value
        self.end_effector_link     = self.get_parameter('end_effector_link').value
        self.reference_frame       = self.get_parameter('reference_frame').value
        self.vel                   = float(self.get_parameter('vel').value)
        self.acc                   = float(self.get_parameter('acc').value)
        self.delay_between_moves   = float(self.get_parameter('delay_between_moves').value)
        self.position_tolerance    = float(self.get_parameter('position_tolerance').value)
        self.orientation_tolerance = float(self.get_parameter('orientation_tolerance').value)

        # Build pose A
        self.pose_a = Pose(
            position=Point(
                x=float(self.get_parameter('position_a.x').value),
                y=float(self.get_parameter('position_a.y').value),
                z=float(self.get_parameter('position_a.z').value),
            ),
            orientation=rpy_to_quaternion(
                self.get_parameter('position_a.roll').value,
                self.get_parameter('position_a.pitch').value,
                self.get_parameter('position_a.yaw').value,
            )
        )

        # Build pose B
        self.pose_b = Pose(
            position=Point(
                x=float(self.get_parameter('position_b.x').value),
                y=float(self.get_parameter('position_b.y').value),
                z=float(self.get_parameter('position_b.z').value),
            ),
            orientation=rpy_to_quaternion(
                self.get_parameter('position_b.roll').value,
                self.get_parameter('position_b.pitch').value,
                self.get_parameter('position_b.yaw').value,
            )
        )

        self.get_logger().info(f'Planning group      : {self.planning_group}')
        self.get_logger().info(f'End effector link   : {self.end_effector_link}')
        self.get_logger().info(f'Reference frame     : {self.reference_frame}')
        self.get_logger().info(f'Velocity scale      : {self.vel}')
        self.get_logger().info(f'Accel scale         : {self.acc}')
        self.get_logger().info(f'Delay between moves : {self.delay_between_moves}s')
        self.get_logger().info(
            f'Pose A : ({self.pose_a.position.x:.3f}, '
            f'{self.pose_a.position.y:.3f}, '
            f'{self.pose_a.position.z:.3f})'
        )
        self.get_logger().info(
            f'Pose B : ({self.pose_b.position.x:.3f}, '
            f'{self.pose_b.position.y:.3f}, '
            f'{self.pose_b.position.z:.3f})'
        )

        # ── MoveGroup action client ──────────────────────────────────────────
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

        # ── State tracking ───────────────────────────────────────────────────
        self.current_target     = 'A'
        self.motion_in_progress = False
        self.iteration          = 0

        # ── Start loop ───────────────────────────────────────────────────────
        self.create_timer(1.0, self.loop_tick)

    def loop_tick(self):
        """
        Called by timer every second.
        Sends next goal only when no motion is in progress.
        """
        if not self.motion_in_progress:
            self.motion_in_progress = True
            pose = self.pose_a if self.current_target == 'A' else self.pose_b
            self.get_logger().info('─' * 40)
            self.get_logger().info(
                f'Iteration {self.iteration + 1} — Moving to Pose {self.current_target}'
            )
            self.send_goal(pose)

    def build_pose_constraints(self, target_pose):
        """
        Build MoveIt position + orientation constraints from a target Pose.

        Two constraints are needed for a full Cartesian goal:
        1. PositionConstraint  — where the TCP should be in space (X, Y, Z)
        2. OrientationConstraint — how the TCP should be oriented (quaternion)

        The position constraint uses a small sphere around the target point.
        MoveIt will plan so the TCP lands inside that sphere.
        """
        constraints = Constraints()
        header = Header()
        header.frame_id = self.reference_frame

        # ── Position constraint ──────────────────────────────────────────────
        # Define a small sphere around the target position.
        # MoveIt plans so the TCP ends up inside this sphere.
        position_constraint                        = PositionConstraint()
        position_constraint.header                 = header
        position_constraint.link_name              = self.end_effector_link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        # Bounding sphere around target position
        bounding_volume              = BoundingVolume()
        sphere                       = SolidPrimitive()
        sphere.type                  = SolidPrimitive.SPHERE
        sphere.dimensions            = [self.position_tolerance]
        bounding_volume.primitives.append(sphere)
        bounding_volume.primitive_poses.append(target_pose)
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight            = 1.0

        # ── Orientation constraint ───────────────────────────────────────────
        # Tell MoveIt the target orientation of the TCP.
        orientation_constraint                            = OrientationConstraint()
        orientation_constraint.header                     = header
        orientation_constraint.link_name                  = self.end_effector_link
        orientation_constraint.orientation                = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation_constraint.weight                     = 1.0

        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def send_goal(self, target_pose):
        """
        Build and send a Cartesian pose goal to MoveIt2.
        """
        self.get_logger().info('Waiting for /move_action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('/move_action server found.')

        goal_msg                    = MoveGroup.Goal()
        goal_msg.request            = MotionPlanRequest()
        goal_msg.request.group_name = self.planning_group

        # Build and attach pose constraints
        goal_msg.request.goal_constraints.append(
            self.build_pose_constraints(target_pose)
        )

        goal_msg.request.max_velocity_scaling_factor     = self.vel
        goal_msg.request.max_acceleration_scaling_factor = self.acc

        goal_msg.planning_options           = PlanningOptions()
        goal_msg.planning_options.plan_only = False

        self.get_logger().info('Sending Cartesian goal to MoveIt...')
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when MoveIt accepts or rejects the goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                'Goal REJECTED. Check that the pose is reachable '
                'and not in collision.'
            )
            self.motion_in_progress = False
            return

        self.get_logger().info('Goal ACCEPTED — executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Called when motion finishes. Flips target and waits for delay."""
        result     = future.result().result
        error_code = result.error_code.val

        if error_code == 1:
            self.get_logger().info(f'Pose {self.current_target} reached. ✓')
        else:
            self.get_logger().error(
                f'Motion FAILED (error code: {error_code}). '
                f'The pose may be unreachable or in collision. '
                f'Try adjusting X/Y/Z values in the YAML.'
            )

        self.iteration += 1

        # Flip target
        self.current_target = 'B' if self.current_target == 'A' else 'A'

        self.get_logger().info(
            f'Waiting {self.delay_between_moves}s before next move...'
        )
        time.sleep(self.delay_between_moves)

        self.motion_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    node = MoveCartesianNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Loop stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
