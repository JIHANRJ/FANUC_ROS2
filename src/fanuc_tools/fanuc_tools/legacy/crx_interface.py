"""
crx_interface.py
----------------
CRX Python interface
- RViz-style planning
- Explicit start state
- Marker visualization
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
    PlanningOptions,
    RobotState
)

from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

import math


# ----------------------------
# WPR → Quaternion
# ----------------------------
def wpr_to_quaternion(w, p, r):
    cy = math.cos(w * 0.5)
    sy = math.sin(w * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


# ==========================================================
# CRX CLASS
# ==========================================================
class CRX(Node):

    def __init__(self):
        super().__init__('crx_interface')

        # ---- THIS MUST EXIST ----
        self.current_joint_state = None

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # MoveIt action client
        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.client.wait_for_server()

        # Marker publisher
        self.marker_pub = self.create_publisher(
            Marker,
            '/crx_points',
            10
        )

    # ------------------------------------------------------
    # Joint State Callback
    # ------------------------------------------------------
    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    # ------------------------------------------------------
    # Build Start State (like RViz)
    # ------------------------------------------------------
    def build_start_state(self):
        state = RobotState()

        if self.current_joint_state is None:
            return state

        js = JointState()
        js.name = list(self.current_joint_state.name)
        js.position = list(self.current_joint_state.position)
        js.velocity = list(self.current_joint_state.velocity)
        js.effort = list(self.current_joint_state.effort)

        state.joint_state = js
        state.is_diff = False
        return state

    # ------------------------------------------------------
    # Joint Move (PTP)
    # ------------------------------------------------------
    def move_joint(self, joints, vel=0.3, acc=0.3):

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = "manipulator"
        goal.request.max_velocity_scaling_factor = vel
        goal.request.max_acceleration_scaling_factor = acc
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 10

        goal.request.start_state = self.build_start_state()

        constraints = Constraints()
        joint_names = ["J1","J2","J3","J4","J5","J6"]

        for name, position in zip(joint_names, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False

        self._execute_goal(goal)

    # ------------------------------------------------------
    # Pose Move (RViz-style)
    # ------------------------------------------------------
    def move_pose(self, pose, vel=0.3, acc=0.3):

        x, y, z, w, p, r = pose
        qx, qy, qz, qw = wpr_to_quaternion(w, p, r)

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = "manipulator"
        goal.request.max_velocity_scaling_factor = vel
        goal.request.max_acceleration_scaling_factor = acc
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 10

        goal.request.start_state = self.build_start_state()

        constraints = Constraints()

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        # Position constraint
        pos = PositionConstraint()
        pos.header.frame_id = "base_link"
        pos.link_name = "flange"

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]

        pos.constraint_region.primitives.append(sphere)
        pos.constraint_region.primitive_poses.append(pose_msg.pose)
        pos.weight = 1.0

        # Orientation constraint
        ori = OrientationConstraint()
        ori.header.frame_id = "base_link"
        ori.link_name = "flange"
        ori.orientation = pose_msg.pose.orientation
        ori.absolute_x_axis_tolerance = 0.001
        ori.absolute_y_axis_tolerance = 0.001
        ori.absolute_z_axis_tolerance = 0.001
        ori.weight = 1.0

        constraints.position_constraints.append(pos)
        constraints.orientation_constraints.append(ori)

        goal.request.goal_constraints.append(constraints)

        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False

        self._execute_goal(goal)

    # ------------------------------------------------------
    # Marker Visualization
    # ------------------------------------------------------
    def show_point(self, pose, name, color):

        x, y, z, w, p, r = pose
        qx, qy, qz, qw = wpr_to_quaternion(w, p, r)

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "crx_points"
        marker.id = abs(hash(name)) % 1000
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        marker.lifetime.sec = 0 

        self.marker_pub.publish(marker)

    # ------------------------------------------------------
    # Execute Helper
    # ------------------------------------------------------
    def _execute_goal(self, goal):
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)


from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def add_box(self, name, size, pose):

    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose

    collision = CollisionObject()
    collision.header.frame_id = "base_link"
    collision.id = name

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = size  # [x, y, z]

    box_pose = Pose()
    box_pose.position.x = pose[0]
    box_pose.position.y = pose[1]
    box_pose.position.z = pose[2]
    box_pose.orientation.w = 1.0

    collision.primitives.append(box)
    collision.primitive_poses.append(box_pose)
    collision.operation = CollisionObject.ADD

    pub = self.create_publisher(CollisionObject, "/collision_object", 10)
    pub.publish(collision)