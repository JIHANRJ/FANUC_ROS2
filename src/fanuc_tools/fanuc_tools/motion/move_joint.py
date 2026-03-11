"""
move_joint.py
=============
Quick run:
    source /opt/ros/humble/setup.bash && source ~/ws_fanuc/install/setup.bash
    ros2 launch fanuc_tools move_joint.launch.py use_mock:=true use_rviz:=false
    ros2 run fanuc_tools speed_scaling

Example: Move the FANUC CRX-10iA/L back and forth between two joint
         configurations in a continuous loop.

What it does:
    Reads two target joint configurations (A and B) from parameters,
    then continuously moves between them with a configurable delay,
    until interrupted with Ctrl+C.

When to use it:
    - Testing motion on real or mock hardware
    - Demonstrating speed scaling alongside speed_scaling.py
    - Verifying joint limits and reachability

Key concepts:
    - MoveGroup action: MoveIt2 interface for planning + execution
    - JointConstraint: target position for each joint
    - Loop with delay: move A→B, wait, move B→A, wait, repeat

Run alongside speed_scaling.py to control speed during motion:
    Terminal 1: ros2 launch fanuc_moveit_config fanuc_moveit.launch.py
                    robot_model:=crx10ia_l use_mock:=true
    Terminal 2: ros2 run fanuc_tools move_joint
    Terminal 3: ros2 run fanuc_tools speed_scaling

Usage:
    ros2 launch fanuc_tools move_joint.launch.py use_mock:=true
"""

import time
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
    Moves the FANUC CRX-10iA/L back and forth between two joint
    configurations loaded from parameters.
    """

    def __init__(self):
        super().__init__('move_joint_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('vel', 0.1)
        self.declare_parameter('acc', 0.1)
        self.declare_parameter('delay_between_moves', 2.0)  # seconds

        # Position A — first target
        self.declare_parameter('position_a.joint_1', 0.0)
        self.declare_parameter('position_a.joint_2', 0.0)
        self.declare_parameter('position_a.joint_3', 0.0)
        self.declare_parameter('position_a.joint_4', 0.0)
        self.declare_parameter('position_a.joint_5', 0.0)
        self.declare_parameter('position_a.joint_6', 0.0)

        # Position B — second target
        self.declare_parameter('position_b.joint_1', 0.0)
        self.declare_parameter('position_b.joint_2', 0.0)
        self.declare_parameter('position_b.joint_3', 0.0)
        self.declare_parameter('position_b.joint_4', 0.0)
        self.declare_parameter('position_b.joint_5', 0.0)
        self.declare_parameter('position_b.joint_6', 0.0)

        # ── Read parameters ──────────────────────────────────────────────────
        self.planning_group      = self.get_parameter('planning_group').value
        self.vel                 = float(self.get_parameter('vel').value)
        self.acc                 = float(self.get_parameter('acc').value)
        self.delay_between_moves = float(self.get_parameter('delay_between_moves').value)

        self.position_a = [
            self.get_parameter('position_a.joint_1').value,
            self.get_parameter('position_a.joint_2').value,
            self.get_parameter('position_a.joint_3').value,
            self.get_parameter('position_a.joint_4').value,
            self.get_parameter('position_a.joint_5').value,
            self.get_parameter('position_a.joint_6').value,
        ]

        self.position_b = [
            self.get_parameter('position_b.joint_1').value,
            self.get_parameter('position_b.joint_2').value,
            self.get_parameter('position_b.joint_3').value,
            self.get_parameter('position_b.joint_4').value,
            self.get_parameter('position_b.joint_5').value,
            self.get_parameter('position_b.joint_6').value,
        ]

        self.get_logger().info(f'Planning group      : {self.planning_group}')
        self.get_logger().info(f'Velocity scale      : {self.vel}')
        self.get_logger().info(f'Accel scale         : {self.acc}')
        self.get_logger().info(f'Delay between moves : {self.delay_between_moves}s')
        self.get_logger().info(f'Position A          : {self.position_a}')
        self.get_logger().info(f'Position B          : {self.position_b}')

        # ── Joint names for CRX-10iA/L ───────────────────────────────────────
        self.joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']

        # ── MoveGroup action client ──────────────────────────────────────────
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

        # ── State tracking ───────────────────────────────────────────────────
        # Tracks which position we're going to next
        self.current_target = 'A'
        self.motion_in_progress = False
        self.iteration = 0

        # ── Start loop ───────────────────────────────────────────────────────
        # Wait 1 second for MoveIt to be ready, then start the loop
        self.create_timer(1.0, self.loop_tick)

    def loop_tick(self):
        """
        Called by timer every second.
        Only sends a new goal when no motion is currently in progress.
        This prevents overlapping goals being sent to MoveIt.
        """
        if not self.motion_in_progress:
            self.motion_in_progress = True
            target = self.position_a if self.current_target == 'A' else self.position_b
            self.get_logger().info('─' * 40)
            self.get_logger().info(
                f'Iteration {self.iteration + 1} — Moving to Position {self.current_target}'
            )
            self.send_goal(target)

    def send_goal(self, target_joints):
        """
        Build and send a MoveGroup goal to MoveIt2.
        """
        self.get_logger().info('Waiting for /move_action server...')
        self._action_client.wait_for_server()

        # Build goal
        goal_msg                 = MoveGroup.Goal()
        goal_msg.request         = MotionPlanRequest()
        goal_msg.request.group_name = self.planning_group

        # Build joint constraints
        constraints = Constraints()
        for name, position in zip(self.joint_names, target_joints):
            jc                   = JointConstraint()
            jc.joint_name        = name
            jc.position          = position
            jc.tolerance_above   = 0.01
            jc.tolerance_below   = 0.01
            jc.weight            = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.request.max_velocity_scaling_factor     = self.vel
        goal_msg.request.max_acceleration_scaling_factor = self.acc

        goal_msg.planning_options            = PlanningOptions()
        goal_msg.planning_options.plan_only  = False

        self.get_logger().info(f'Sending goal to MoveIt...')
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when MoveIt accepts or rejects the goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED. Check joint values.')
            self.motion_in_progress = False
            return

        self.get_logger().info('Goal ACCEPTED — executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """
        Called when motion finishes.
        Flips the target (A→B or B→A), waits the configured delay,
        then allows the next move to start.
        """
        result     = future.result().result
        error_code = result.error_code.val

        if error_code == 1:
            self.get_logger().info(
                f'Position {self.current_target} reached. ✓'
            )
        else:
            self.get_logger().error(
                f'Motion FAILED (error code: {error_code})'
            )

        self.iteration += 1

        # Flip target for next move
        self.current_target = 'B' if self.current_target == 'A' else 'A'

        # Wait the configured delay before allowing next move
        self.get_logger().info(
            f'Waiting {self.delay_between_moves}s before next move...'
        )
        time.sleep(self.delay_between_moves)

        # Allow loop_tick to send the next goal
        self.motion_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    node = MoveJointNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Loop stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
