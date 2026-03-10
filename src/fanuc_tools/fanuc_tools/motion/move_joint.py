"""
move_joint.py
=============
Example: Move the FANUC CRX-10iA/L to a target joint configuration.

What it does:
    Reads target joint positions from ROS 2 parameters (move_joint.yaml),
    sends a MoveGroup action goal to MoveIt2, and executes it.

When to use it:
    Use joint-space motion when you only care about the destination pose,
    not the path the robot takes to get there. Fastest and most reliable
    motion type.

Key concepts:
    - MoveGroup action: the main MoveIt2 interface for planning + execution
    - JointConstraint: tells MoveIt the target position for each joint
    - /move_action: the action server that MoveIt exposes

How it differs from moveit_joint.py:
    - Loads joint targets from YAML parameters instead of command line args
    - Includes connection status check before moving
    - More detailed logging at each step

Usage:
    ros2 launch fanuc_tools move_joint.launch.py use_mock:=true

Run directly (without launch file):
    First launch MoveIt separately:
    ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \
        robot_model:=crx10ia_l use_mock:=true

    Then run:
    source ~/ws_fanuc/install/setup.bash
        ros2 run fanuc_tools move_joint --ros-args \
        -p planning_group:=manipulator \
        -p vel:=0.1 \
        -p acc:=0.1 \
        -p target_joints.joint_1:=0.0 \
        -p target_joints.joint_2:=-0.7854 \
        -p target_joints.joint_3:=1.5708 \
        -p target_joints.joint_4:=0.0 \
        -p target_joints.joint_5:=0.7854 \
        -p target_joints.joint_6:=0.0


"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
)


class MoveJointNode(Node):
    """
    Moves the FANUC CRX-10iA/L to a target joint configuration
    by sending a goal to MoveIt2's /move_action server.
    """

    def __init__(self):
        super().__init__('move_joint_node')

        # ── Parameters ─────────────────────────────────────────────────────
        # Loaded from config/motion/move_joint.yaml
        # Safe defaults declared here in case YAML is missing
        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('vel', 0.1)   # 10% velocity — safe for testing
        self.declare_parameter('acc', 0.1)   # 10% acceleration — safe for testing

        # Target joint positions in radians (6 joints for CRX-10iA/L)
        self.declare_parameter('target_joints.joint_1', 0.0)
        self.declare_parameter('target_joints.joint_2', 0.0)
        self.declare_parameter('target_joints.joint_3', 0.0)
        self.declare_parameter('target_joints.joint_4', 0.0)
        self.declare_parameter('target_joints.joint_5', 0.0)
        self.declare_parameter('target_joints.joint_6', 0.0)

        # ── Read parameters ─────────────────────────────────────────────────
        self.planning_group = self.get_parameter('planning_group').value
        self.vel = float(self.get_parameter('vel').value)
        self.acc = float(self.get_parameter('acc').value)

        self.target_joints = [
            self.get_parameter('target_joints.joint_1').value,
            self.get_parameter('target_joints.joint_2').value,
            self.get_parameter('target_joints.joint_3').value,
            self.get_parameter('target_joints.joint_4').value,
            self.get_parameter('target_joints.joint_5').value,
            self.get_parameter('target_joints.joint_6').value,
        ]

        self.get_logger().info(f'Planning group : {self.planning_group}')
        self.get_logger().info(f'Velocity scale : {self.vel}')
        self.get_logger().info(f'Accel scale    : {self.acc}')
        self.get_logger().info(f'Target joints  : {self.target_joints}')

        # ── Joint names for CRX-10iA/L ──────────────────────────────────────
        # These must match the joint names in the URDF exactly.
        # Verify with: ros2 topic echo /joint_states --once
        self.joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']

        # ── MoveGroup action client ─────────────────────────────────────────
        # /move_action is the MoveIt2 action server for planning + execution
        self._action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        # ── One-shot timer ──────────────────────────────────────────────────
        # Wait 1 second after startup before sending goal.
        # Gives MoveIt time to fully initialise before we connect.
        self.sent = False
        self.create_timer(1.0, self.start_once)

    def start_once(self):
        """Called by timer — ensures goal is only sent once."""
        if not self.sent:
            self.sent = True
            self.send_goal()

    def send_goal(self):
        """
        Build and send a MoveGroup goal to MoveIt2.

        The goal contains:
        - group_name: which planning group to use
        - goal_constraints: target joint positions as JointConstraints
        - max_velocity/acceleration_scaling_factor: speed limits
        - plan_only=False: plan AND execute (not just plan)
        """
        self.get_logger().info('Waiting for /move_action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('/move_action server found.')

        # Build the MoveGroup goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()

        # Which planning group to use
        goal_msg.request.group_name = self.planning_group

        # Build joint constraints — one per joint
        # Each JointConstraint tells MoveIt:
        #   "I want joint X to be at position Y, within tolerance Z"
        constraints = Constraints()
        for name, position in zip(self.joint_names, self.target_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01  # radians
            jc.tolerance_below = 0.01  # radians
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)

        # Speed limits — keep low for first tests
        goal_msg.request.max_velocity_scaling_factor = self.vel
        goal_msg.request.max_acceleration_scaling_factor = self.acc

        # plan_only=False means plan AND execute
        # Set to True if you only want to preview the path in RViz
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False

        self.get_logger().info('Sending joint goal to MoveIt...')
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when MoveIt accepts or rejects the goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                'Goal REJECTED by MoveIt. '
                'Check joint values are within limits.'
            )
            rclpy.shutdown()
            return

        self.get_logger().info('Goal ACCEPTED. Robot is moving...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Called when motion is complete."""
        result = future.result().result
        error_code = result.error_code.val

        # MoveIt error code 1 = SUCCESS
        if error_code == 1:
            self.get_logger().info('Motion complete. ✓')
        else:
            self.get_logger().error(
                f'Motion FAILED with error code: {error_code}. '
                f'Check MoveIt logs for details.'
            )

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MoveJointNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
