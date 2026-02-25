"""
moveit_cartesian.py
-------------------
Cartesian MoveIt control using XYZ-WPR.

Pose format:
    [x, y, z, w, p, r]  (meters, radians)
    w = yaw (Z)
    p = pitch (Y)
    r = roll (X)

Planning group: "manipulator"
Frame: base_link → flange

Build:
    cd ~/ws_fanuc
    colcon build --symlink-install
    source install/setup.bash

Run (simulation):
    Termianl 1:
    ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \
        robot_model:=crx10ia_l use_mock:=true
    Terminal 2:
    ros2 run fanuc_tools moveit_cartesian \
        --ros-args \
        -p pose:="[0.6,0.0,0.8,0.0,1.57,0.0]" \
        -p vel:=0.3 -p acc:=0.3

Run (real robot):
    use_mock:=false + robot_ip:=<IP>
    Start with vel:=0.1 acc:=0.1

Units:
    meters, radians
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive


# Convert WPR (yaw, pitch, roll) to quaternion (ZYX)
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


class MoveItWPR(Node):

    def __init__(self):
        super().__init__('moveit_wpr')

        self.declare_parameter("pose", [0.6, 0.0, 0.8, 0.0, 1.57, 0.0])
        self.declare_parameter("vel", 0.3)
        self.declare_parameter("acc", 0.3)

        self.pose = self.get_parameter("pose").value
        self.vel = float(self.get_parameter("vel").value)
        self.acc = float(self.get_parameter("acc").value)

        if len(self.pose) != 6:
            raise ValueError("Pose must be [x,y,z,w,p,r]")

        self.get_logger().info(f"Target XYZ-WPR: {self.pose}")

        self._action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.timer = self.create_timer(1.0, self.start_once)
        self.sent = False

    def start_once(self):
        if self.sent:
            return
        self.sent = True
        self.send_goal()

    def send_goal(self):

        self.get_logger().info("Waiting for move_action server...")
        self._action_client.wait_for_server()

        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = "manipulator"

        x, y, z, w, p, r = self.pose
        qx, qy, qz, qw = wpr_to_quaternion(w, p, r)

        # ---------------- Position Constraint ----------------
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "flange"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]

        pos_constraint.constraint_region.primitives.append(box)

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        pos_constraint.constraint_region.primitive_poses.append(pose.pose)
        pos_constraint.weight = 1.0

        # ---------------- Orientation Constraint ----------------
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "base_link"
        ori_constraint.link_name = "flange"
        ori_constraint.orientation.x = qx
        ori_constraint.orientation.y = qy
        ori_constraint.orientation.z = qz
        ori_constraint.orientation.w = qw
        ori_constraint.absolute_x_axis_tolerance = 0.01
        ori_constraint.absolute_y_axis_tolerance = 0.01
        ori_constraint.absolute_z_axis_tolerance = 0.01
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)

        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.request.max_velocity_scaling_factor = self.vel
        goal_msg.request.max_acceleration_scaling_factor = self.acc

        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False

        self.get_logger().info("Sending XYZ-WPR goal...")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info("Motion complete.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MoveItWPR()
    rclpy.spin(node)


if __name__ == '__main__':
    main()