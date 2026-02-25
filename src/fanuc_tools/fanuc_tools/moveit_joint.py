"""
moveit_joint.py
---------------
Joint-space MoveIt control (J1–J6).

Parameters:
    joints -> [J1,J2,J3,J4,J5,J6] (radians)
    vel    -> velocity scaling (0.0–1.0)
    acc    -> acceleration scaling (0.0–1.0)

Planning group: "manipulator"

Build:
    cd ~/ws_fanuc
    colcon build --symlink-install
    source install/setup.bash

Run (simulation):
    ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \
        robot_model:=crx10ia_l use_mock:=true

    ros2 run fanuc_tools moveit_joint \
        --ros-args \
        -p joints:="[0.3,0.4,-1.2,1.2,-0.3,0.5]" \
        -p vel:=0.3 -p acc:=0.3

Run (real robot):
    use_mock:=false + robot_ip:=<IP>
    Start with vel:=0.1 acc:=0.1

Units:
    radians
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions


class MoveItJoint(Node):

    def __init__(self):
        super().__init__('moveit_joint')

        self.declare_parameter("joints", [0.0]*6)
        self.declare_parameter("vel", 0.3)
        self.declare_parameter("acc", 0.3)

        self.joints = self.get_parameter("joints").value
        self.vel = float(self.get_parameter("vel").value)
        self.acc = float(self.get_parameter("acc").value)

        if len(self.joints) != 6:
            raise ValueError("You must provide exactly 6 joint values.")

        self._action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.timer = self.create_timer(1.0, self.start_once)
        self.sent = False

    def start_once(self):
        if not self.sent:
            self.sent = True
            self.send_goal()

    def send_goal(self):

        self.get_logger().info("Waiting for move_action server...")
        self._action_client.wait_for_server()

        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = "manipulator"

        joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]

        constraints = Constraints()

        for name, position in zip(joint_names, self.joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.request.max_velocity_scaling_factor = self.vel
        goal_msg.request.max_acceleration_scaling_factor = self.acc

        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False

        self.get_logger().info("Sending joint goal...")
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

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
        self.get_logger().info("Joint motion complete.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MoveItJoint()
    rclpy.spin(node)


if __name__ == '__main__':
    main()