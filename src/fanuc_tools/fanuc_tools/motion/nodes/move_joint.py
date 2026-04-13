"""
move_joint.py
=============
Quick run:
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
    - Reusing the modular `JointMotionClient` from other scripts

Key concepts:
    - MoveGroup action: MoveIt2 interface for planning + execution
    - JointConstraint: target position for each joint
    - Loop with delay: move A→B, wait, move B→A, wait, repeat
    - Modular API: import helpers from `fanuc_tools.motion.core.joint_motion`

Run alongside speed_scaling.py to control speed during motion:
    Terminal 1: ros2 launch fanuc_moveit_config fanuc_moveit.launch.py
                    robot_model:=crx10ia_l use_mock:=true
    Terminal 2: ros2 run fanuc_tools move_joint
    Terminal 3: ros2 run fanuc_tools speed_scaling

Usage:
    ros2 launch fanuc_tools move_joint.launch.py use_mock:=true

Parameter method:
    1) Pass parameters directly in launch command:
       ros2 launch fanuc_tools move_joint.launch.py \
           use_mock:=true \
           vel:=0.2 acc:=0.2 delay_between_moves:=1.5 \
           position_a.joint_1:=0.0 position_a.joint_2:=0.0 position_a.joint_3:=0.0 \
           position_a.joint_4:=0.0 position_a.joint_5:=0.0 position_a.joint_6:=0.0 \
           position_b.joint_1:=15.0 position_b.joint_2:=-20.0 position_b.joint_3:=35.0 \
           position_b.joint_4:=0.0 position_b.joint_5:=10.0 position_b.joint_6:=0.0

    2) Or load from YAML using --ros-args:
       ros2 run fanuc_tools move_joint --ros-args --params-file /path/to/move_joint_params.yaml
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from fanuc_tools.motion.core.joint_motion import (
    GOAL_REJECTED_ERROR_CODE,
    JointMotionClient,
    degrees_to_radians,
    radians_to_degrees,
)


@dataclass(frozen=True)
class MoveJointLoopConfig:
    """Standalone loop configuration for alternating between positions A and B."""

    planning_group: str = 'manipulator'
    vel: float = 0.1
    acc: float = 0.1
    startup_delay: float = 5.0
    delay_between_moves: float = 2.0
    position_a_deg: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    position_b_deg: tuple[float, ...] = (30.0, -25.0, 40.0, 0.0, 15.0, 0.0)

    @classmethod
    def from_node_parameters(cls, node: Node) -> 'MoveJointLoopConfig':
        """Load the standalone loop configuration from ROS parameters."""
        declare_move_joint_parameters(node)
        return cls(
            planning_group=node.get_parameter('planning_group').value,
            vel=float(node.get_parameter('vel').value),
            acc=float(node.get_parameter('acc').value),
            startup_delay=float(node.get_parameter('startup_delay').value),
            delay_between_moves=float(node.get_parameter('delay_between_moves').value),
            position_a_deg=tuple(read_joint_vector_degrees(node, 'position_a')),
            position_b_deg=tuple(read_joint_vector_degrees(node, 'position_b')),
        )

def declare_move_joint_parameters(node: Node) -> None:
    """Declare all parameters used by the standalone `move_joint` node."""
    node.declare_parameter('planning_group', 'manipulator')
    node.declare_parameter('vel', 0.1)
    node.declare_parameter('acc', 0.1)
    node.declare_parameter('delay_between_moves', 2.0)
    node.declare_parameter('startup_delay', 5.0)

    position_a_defaults = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    position_b_defaults = (30.0, -25.0, 40.0, 0.0, 15.0, 0.0)

    for index, default in enumerate(position_a_defaults, start=1):
        node.declare_parameter(f'position_a.joint_{index}', default)

    for index, default in enumerate(position_b_defaults, start=1):
        node.declare_parameter(f'position_b.joint_{index}', default)


def read_joint_vector_degrees(node: Node, prefix: str) -> list[float]:
    """Read `position_a.*` or `position_b.*` parameters in degrees."""
    return [
        float(node.get_parameter(f'{prefix}.joint_{index}').value)
        for index in range(1, 7)
    ]


class MoveJointNode(Node):
    """
    Standalone example node that alternates between two joint targets.

    For reusable programmatic control, import `JointMotionClient` from
    `fanuc_tools.motion.core.joint_motion` instead of this looping example node.
    """

    def __init__(self):
        super().__init__('move_joint_node')

        self.config = MoveJointLoopConfig.from_node_parameters(self)
        self.motion_client = JointMotionClient(
            self,
            planning_group=self.config.planning_group,
            vel=self.config.vel,
            acc=self.config.acc,
        )

        self.position_a = degrees_to_radians(self.config.position_a_deg)
        self.position_b = degrees_to_radians(self.config.position_b_deg)

        self.current_target = 'B'
        self.motion_in_progress = False
        self.iteration = 0
        self.start_time = time.monotonic()
        self.next_move_time = self.start_time + self.config.startup_delay
        self.startup_wait_logged = False

        self._log_configuration()
        self.create_timer(0.25, self.loop_tick)

    def _log_configuration(self) -> None:
        self.get_logger().info(f'Planning group      : {self.config.planning_group}')
        self.get_logger().info(f'Velocity scale      : {self.config.vel}')
        self.get_logger().info(f'Accel scale         : {self.config.acc}')
        self.get_logger().info(f'Startup delay       : {self.config.startup_delay}s')
        self.get_logger().info(
            f'Delay between moves : {self.config.delay_between_moves}s'
        )
        self.get_logger().info(f'Position A (deg)    : {list(self.config.position_a_deg)}')
        self.get_logger().info(f'Position B (deg)    : {list(self.config.position_b_deg)}')

    def loop_tick(self) -> None:
        """Send the next goal when startup delay and dwell time have passed."""
        now = time.monotonic()
        startup_phase = now - self.start_time < self.config.startup_delay

        if now < self.next_move_time:
            if startup_phase and not self.startup_wait_logged:
                self.get_logger().info(
                    f'Waiting {self.config.startup_delay:.1f}s startup delay '
                    'for MoveIt/controllers...'
                )
                self.startup_wait_logged = True
            return

        if self.motion_in_progress:
            return

        self.motion_in_progress = True
        target = self.position_a if self.current_target == 'A' else self.position_b
        self.get_logger().info('─' * 40)
        self.get_logger().info(
            f'Iteration {self.iteration + 1} — Moving to Position {self.current_target}'
        )
        target_deg = [round(value, 1) for value in radians_to_degrees(target)]
        self.get_logger().info(f'Target (deg)        : {target_deg}')
        self.send_goal(target)

    def send_goal(self, target_joints):
        """Send a joint goal in radians through the reusable `JointMotionClient`."""
        self.get_logger().info('Waiting for /move_action server...')
        self.motion_client.wait_for_server()
        self.get_logger().info('Sending goal to MoveIt...')
        return self.motion_client.send_goal(
            target_joints,
            goal_response_callback=self.goal_response_callback,
            result_callback=self.result_callback,
        )

    def goal_response_callback(self, goal_handle) -> None:
        """Called when MoveIt accepts or rejects the goal."""
        if goal_handle.accepted:
            self.get_logger().info('Goal ACCEPTED — executing...')

    def result_callback(self, error_code: int, _result) -> None:
        """Called when motion finishes or the goal is rejected."""
        if error_code == 1:
            self.get_logger().info(f'Position {self.current_target} reached. ✓')
        elif error_code == GOAL_REJECTED_ERROR_CODE:
            self.get_logger().error('Goal REJECTED. Check joint values.')
        else:
            self.get_logger().error(f'Motion FAILED (error code: {error_code})')

        self.iteration += 1
        self.current_target = 'B' if self.current_target == 'A' else 'A'
        self.next_move_time = time.monotonic() + self.config.delay_between_moves
        self.motion_in_progress = False
        self.get_logger().info(
            f'Waiting {self.config.delay_between_moves}s before next move...'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MoveJointNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Loop stopped by user.')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    main()
