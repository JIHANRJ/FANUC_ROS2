"""
move_joint.py
=============
Example: Move the FANUC CRX-10iA/L to a target joint configuration using MoveIt2.

What it does:
    Reads target joint positions from ROS 2 parameters (move_joint.yaml),
    plans a trajectory to that configuration using MoveIt2, and executes it
    on the real robot.

When to use it:
    Use joint-space motion when you only care about the destination pose,
    not the path the robot takes to get there. It's the fastest and most
    reliable motion type.

Key concepts:
    - MoveItPy: the MoveIt2 Python API
    - Planning component: the group of joints being planned for ("manipulator")
    - Speed scaling: published to /speed_scaling_factor to safely limit speed

ROS 2 Topics used:
    /speed_scaling_factor  [std_msgs/Float64]  -- limits execution speed 0-100

Usage:
    ros2 launch fanuc_tools move_joint.launch.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# MoveIt2 Python API
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


class MoveJointNode(Node):
    """
    A ROS 2 node that moves the FANUC CRX-10iA/L to a target
    joint configuration loaded from parameters.
    """

    def __init__(self):
        super().__init__('move_joint_node')

        # ── Parameters ────────────────────────────────────────────────────────
        # All values come from config/motion/move_joint.yaml
        # Declare them here with safe defaults in case the YAML is missing

        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('robot_model', 'crx10ial')
        self.declare_parameter('speed_scaling', 10.0)  # 10% by default — safe

        # Target joint positions in radians (6 joints for CRX-10iA/L)
        self.declare_parameter('target_joints.joint_1', 0.0)
        self.declare_parameter('target_joints.joint_2', 0.0)
        self.declare_parameter('target_joints.joint_3', 0.0)
        self.declare_parameter('target_joints.joint_4', 0.0)
        self.declare_parameter('target_joints.joint_5', 0.0)
        self.declare_parameter('target_joints.joint_6', 0.0)

        # ── Read parameters ────────────────────────────────────────────────────
        self.planning_group = self.get_parameter('planning_group').value
        self.robot_model    = self.get_parameter('robot_model').value
        self.speed_scaling  = self.get_parameter('speed_scaling').value

        self.target_joints = {
            'joint_1': self.get_parameter('target_joints.joint_1').value,
            'joint_2': self.get_parameter('target_joints.joint_2').value,
            'joint_3': self.get_parameter('target_joints.joint_3').value,
            'joint_4': self.get_parameter('target_joints.joint_4').value,
            'joint_5': self.get_parameter('target_joints.joint_5').value,
            'joint_6': self.get_parameter('target_joints.joint_6').value,
        }

        self.get_logger().info(f'Planning group : {self.planning_group}')
        self.get_logger().info(f'Robot model    : {self.robot_model}')
        self.get_logger().info(f'Speed scaling  : {self.speed_scaling}%')
        self.get_logger().info(f'Target joints  : {self.target_joints}')

        # ── Speed scaling publisher ────────────────────────────────────────────
        # The scaled_joint_trajectory_controller subscribes to this topic.
        # Publishing before motion ensures the robot respects the speed limit
        # from the very first command.
        self.speed_pub = self.create_publisher(
            Float64,
            '/speed_scaling_factor',
            10
        )

        # ── MoveIt2 initialisation ─────────────────────────────────────────────
        # MoveItPy needs the node to be fully initialised before being created.
        # It reads the robot description and planning pipeline from the
        # parameter server (set up by fanuc_moveit.launch.py).
        self.get_logger().info('Initialising MoveItPy...')
        self.moveit = MoveItPy(node_name='move_joint_moveit')
        self.arm    = self.moveit.get_planning_component(self.planning_group)
        self.get_logger().info('MoveItPy ready.')

        # ── Execute motion ─────────────────────────────────────────────────────
        # Use a one-shot timer so the node is fully spun up before we move.
        # This avoids race conditions with MoveIt initialisation.
        self.create_timer(1.0, self.execute_motion)
        self.timer_cancelled = False

    def publish_speed_scaling(self):
        """
        Publish the speed scaling value to the controller.
        Value is 0.0 - 100.0 representing percentage of full speed.
        """
        msg = Float64()
        msg.data = float(self.speed_scaling)
        self.speed_pub.publish(msg)
        self.get_logger().info(f'Speed scaling set to {self.speed_scaling}%')

    def execute_motion(self):
        """
        Plan and execute a joint-space move to the target configuration.
        Called once after node initialisation via a one-shot timer.
        """
        # Guard so this only runs once even if timer fires again
        if self.timer_cancelled:
            return
        self.timer_cancelled = True

        # Step 1: Set speed scaling before sending any motion command
        self.publish_speed_scaling()

        # Step 2: Set the start state to the robot's current position
        # This tells MoveIt where we are right now before planning
        self.arm.set_start_state_to_current_state()

        # Step 3: Build the goal joint state
        robot_model = self.moveit.get_robot_model()
        robot_state = RobotState(robot_model)

        # set_joint_group_positions takes a list in joint order
        robot_state.set_joint_group_positions(
            self.planning_group,
            list(self.target_joints.values())
        )

        # Step 4: Set the goal state in the planning component
        self.arm.set_goal_state(robot_state=robot_state)

        # Step 5: Plan the trajectory
        self.get_logger().info('Planning trajectory...')
        plan_result = self.arm.plan()

        if not plan_result:
            self.get_logger().error(
                'Planning FAILED. Check that the target joints are reachable '
                'and within joint limits.'
            )
            return

        self.get_logger().info('Plan successful. Executing...')

        # Step 6: Execute the planned trajectory on the robot
        # blocking=True means we wait here until execution is complete
        execute_result = self.moveit.execute(
            plan_result.trajectory,
            controllers=[],
            blocking=True
        )

        if execute_result:
            self.get_logger().info('Motion complete.')
        else:
            self.get_logger().error(
                'Execution FAILED. Check driver connection and robot status.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MoveJointNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
