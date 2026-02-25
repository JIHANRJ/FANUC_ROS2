"""
moveit_linear.py
----------------
True linear Cartesian motion (industrial LIN style).

Pose format:
    [x, y, z, w, p, r]  (meters, radians)

Behavior:
    - Computes straight-line Cartesian path
    - Executes ONLY if fraction == 1.0

Run:
    ros2 run fanuc_tools moveit_linear \
      --ros-args \
      -p pose:="[0.7,-0.15,0.955,1.0,0.0,0.0]"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient
import math


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


class MoveItLinear(Node):

    def __init__(self):
        super().__init__('moveit_linear')

        self.declare_parameter("pose", [0.6, 0.0, 0.8, 0.0, 1.57, 0.0])
        self.pose = self.get_parameter("pose").value

        self.cartesian_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )

        self.exec_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )

        while not self.cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for compute_cartesian_path service...")

        self.compute_and_execute()

    def compute_and_execute(self):

        x, y, z, w, p, r = self.pose
        qx, qy, qz, qw = wpr_to_quaternion(w, p, r)

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.x = qx
        target_pose.orientation.y = qy
        target_pose.orientation.z = qz
        target_pose.orientation.w = qw

        request = GetCartesianPath.Request()
        request.group_name = "manipulator"
        request.link_name = "flange"
        request.waypoints.append(target_pose)
        request.max_step = 0.01
        request.jump_threshold = 0.0

        future = self.cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if not response:
            self.get_logger().error("Failed to compute Cartesian path.")
            return

        fraction = response.fraction
        self.get_logger().info(f"Cartesian path fraction: {fraction}")

        if fraction < 1.0:
            self.get_logger().warn("Path not fully achievable. Motion NOT executed.")
            return

        # Execute trajectory
        self.get_logger().info("Executing linear trajectory...")

        while not self.exec_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("Waiting for execute_trajectory action server...")

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution

        send_future = self.exec_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Execution rejected.")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Linear motion complete.")


def main(args=None):
    rclpy.init(args=args)
    node = MoveItLinear()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()