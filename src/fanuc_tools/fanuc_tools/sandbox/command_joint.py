"""
command_joints.py
=================
Send a joint goal to MoveIt.

Usage:
    Terminal 1: ros2 launch fanuc_moveit_config fanuc_moveit.launch.py robot_model:=crx10ia_l use_mock:=true
    Terminal 2: python3 sandbox/command_joints.py
"""

import math
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
from sensor_msgs.msg import JointState

JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']


class JointCommander(Node):

    def __init__(self):
        super().__init__('joint_commander')

        self.current_joints = None
        self.target_joints  = None
        self.is_moving      = False
        self.on_result      = None  # override: fn(error_code, final_joints)

        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )

        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('JointCommander ready.')

    # ── Public API ────────────────────────────────────────────────────────────

    def send_goal(self, target_joints, vel=0.1, acc=0.1):
        """
        Send a joint goal to MoveIt.

        Args:
            target_joints : list[float] — 6 joint positions in radians
            vel           : float       — velocity scaling 0.0-1.0
            acc           : float       — acceleration scaling 0.0-1.0
        """
        if len(target_joints) != 6:
            self.get_logger().error('Need exactly 6 joint values.')
            return

        self.target_joints = target_joints
        self.is_moving     = True

        self._action_client.wait_for_server()

        goal_msg                    = MoveGroup.Goal()
        goal_msg.request            = MotionPlanRequest()
        goal_msg.request.group_name = 'manipulator'
        goal_msg.request.max_velocity_scaling_factor     = float(vel)
        goal_msg.request.max_acceleration_scaling_factor = float(acc)

        constraints = Constraints()
        for name, pos in zip(JOINT_NAMES, target_joints):
            jc                 = JointConstraint()
            jc.joint_name      = name
            jc.position        = float(pos)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options           = PlanningOptions()
        goal_msg.planning_options.plan_only = False

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def get_current_joints(self):
        """
        Returns latest joint positions as dict.

        Returns:
            dict  — {'J1': 0.12, 'J2': -0.45, ...}
            None  — if not received yet
        """
        if self.current_joints is None:
            return None
        return dict(zip(JOINT_NAMES, self.current_joints))

    def get_error_to_target(self):
        """
        Returns per-joint error to current target in radians.

        Returns:
            dict  — {'J1': 0.03, 'J2': 0.12, ...}
            None  — if no target or no joint states yet
        """
        if self.current_joints is None or self.target_joints is None:
            return None
        return {
            name: abs(c - t)
            for name, c, t in zip(
                JOINT_NAMES, self.current_joints, self.target_joints
            )
        }

    def is_ready(self):
        """Returns True if /joint_states has been received."""
        return self.current_joints is not None

    # ── Internal ──────────────────────────────────────────────────────────────

    def _joint_state_callback(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        self.current_joints = [
            name_to_pos.get(n, 0.0) for n in JOINT_NAMES
        ]

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED.')
            self.is_moving = False
            return
        self.get_logger().info('Goal ACCEPTED — moving...')
        goal_handle.get_result_async().add_done_callback(
            self._result_callback
        )

    def _result_callback(self, future):
        error_code     = future.result().result.error_code.val
        self.is_moving = False

        if self.on_result:
            self.on_result(error_code, self.current_joints)
        else:
            self._print_result(error_code)

    def _print_result(self, error_code):
        if error_code == 1:
            self.get_logger().info('✓ MOTION COMPLETE')
            if self.current_joints:
                for name, pos in zip(JOINT_NAMES, self.current_joints):
                    self.get_logger().info(
                        f'  {name}: {pos:+.4f} rad '
                        f'({math.degrees(pos):+.2f} deg)'
                    )
        else:
            self.get_logger().error(f'✗ FAILED — error code: {error_code}')


# ── Standalone runner ─────────────────────────────────────────────────────────

TARGET = [
     0.5236,   # J1 +30 deg
    -0.7854,   # J2 -45 deg
     1.5708,   # J3 +90 deg
     0.0,      # J4   0 deg
     0.7854,   # J5 +45 deg
     0.0,      # J6   0 deg
]


def main():
    rclpy.init()
    node        = JointCommander()
    node._sent  = False

    def send_once():
        if not node._sent:
            node._sent = True
            node.send_goal(TARGET, vel=0.2, acc=0.2)

    node.create_timer(1.0, send_once)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()