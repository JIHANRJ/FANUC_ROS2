"""
moveit_go.py
------------
Proof-of-concept MoveGroup interface (RViz-like behavior).

Modes:
    joint  -> -p joints:="[J1..J6]"
    pose   -> -p pose:="[x,y,z,w,p,r]"

Run (simulation first):
    ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \
        robot_model:=crx10ia_l use_mock:=true

Joint example:
    ros2 run fanuc_tools moveit_go \
      --ros-args \
      -p mode:=joint \
      -p joints:="[0.3,0.4,-1.2,1.2,-0.3,0.5]"

Pose example:
    ros2 run fanuc_tools moveit_go \
      --ros-args \
      -p mode:=pose \
      -p pose:="[0.7,-0.15,0.955,1.0,0.0,0.0]"
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    PlanningOptions
)
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive


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


class MoveItGo(Node):

    def __init__(self):
        super().__init__('moveit_go')

        self.declare_parameter("mode", "joint")
        self.declare_parameter("joints", [0.0]*6)
        self.declare_parameter("pose", [0.6,0.0,0.8,0.0,1.57,0.0])
        self.declare_parameter("vel", 0.3)
        self.declare_parameter("acc", 0.3)
        self.declare_parameter("plan_only", False)

        self.mode = self.get_parameter("mode").value
        self.joints = self.get_parameter("joints").value
        self.pose = self.get_parameter("pose").value
        self.vel = float(self.get_parameter("vel").value)
        self.acc = float(self.get_parameter("acc").value)
        self.plan_only = self.get_parameter("plan_only").value

        self.client = ActionClient(self, MoveGroup, '/move_action')

        self.timer = self.create_timer(1.0, self.start_once)
        self.sent = False

    def start_once(self):
        if not self.sent:
            self.sent = True
            self.send_goal()

    def send_goal(self):

        self.get_logger().info("Waiting for move_action server...")
        self.client.wait_for_server()

        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = "manipulator"
        goal_msg.request.max_velocity_scaling_factor = self.vel
        goal_msg.request.max_acceleration_scaling_factor = self.acc

        constraints = Constraints()

        if self.mode == "joint":
            joint_names = ["J1","J2","J3","J4","J5","J6"]

            for name, position in zip(joint_names, self.joints):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = position
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)

        elif self.mode == "pose":

            x,y,z,w,p,r = self.pose
            qx,qy,qz,qw = wpr_to_quaternion(w,p,r)

            pos = PositionConstraint()
            pos.header.frame_id = "base_link"
            pos.link_name = "flange"

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [0.001,0.001,0.001]
            pos.constraint_region.primitives.append(box)

            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            pos.constraint_region.primitive_poses.append(pose.pose)
            pos.weight = 1.0

            ori = OrientationConstraint()
            ori.header.frame_id = "base_link"
            ori.link_name = "flange"
            ori.orientation = pose.pose.orientation
            ori.absolute_x_axis_tolerance = 0.01
            ori.absolute_y_axis_tolerance = 0.01
            ori.absolute_z_axis_tolerance = 0.01
            ori.weight = 1.0

            constraints.position_constraints.append(pos)
            constraints.orientation_constraints.append(ori)

        else:
            self.get_logger().error("Mode must be 'joint' or 'pose'")
            rclpy.shutdown()
            return

        goal_msg.request.goal_constraints.append(constraints)

        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = self.plan_only

        self.get_logger().info("Sending goal...")
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error("Goal rejected.")
            rclpy.shutdown()
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(self.done)

    def done(self, future):
        self.get_logger().info("Motion complete.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MoveItGo()
    rclpy.spin(node)


if __name__ == '__main__':
    main()