"""
joint_motion.py
===============
Reusable helpers for sending MoveIt joint goals from other motion scripts.

This module keeps the standalone `move_joint.py` behaviour, but exposes a
clean API that other code can import without inheriting the looping example.
"""

from __future__ import annotations

import math
from typing import Callable, Optional, Sequence

from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions


DEFAULT_JOINT_NAMES = ('J1', 'J2', 'J3', 'J4', 'J5', 'J6')
GOAL_REJECTED_ERROR_CODE = 99999


def degrees_to_radians(values_deg: Sequence[float]) -> list[float]:
    """Convert a joint vector from degrees to radians."""
    return [math.radians(float(value)) for value in values_deg]


def radians_to_degrees(values_rad: Sequence[float]) -> list[float]:
    """Convert a joint vector from radians to degrees."""
    return [math.degrees(float(value)) for value in values_rad]


def named_joint_map(
    joint_values: Sequence[float],
    joint_names: Sequence[str] = DEFAULT_JOINT_NAMES,
) -> dict[str, float]:
    """Return `{joint_name: value}` for a joint vector."""
    _validate_joint_vector(joint_values, joint_names)
    return dict(zip(joint_names, joint_values))


def build_joint_constraints(
    target_joints: Sequence[float],
    *,
    joint_names: Sequence[str] = DEFAULT_JOINT_NAMES,
    tolerance_above: float = 0.01,
    tolerance_below: float = 0.01,
    constraint_weight: float = 1.0,
) -> Constraints:
    """Build MoveIt `Constraints` for a target joint vector in radians."""
    _validate_joint_vector(target_joints, joint_names)

    constraints = Constraints()
    for name, position in zip(joint_names, target_joints):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = name
        joint_constraint.position = float(position)
        joint_constraint.tolerance_above = tolerance_above
        joint_constraint.tolerance_below = tolerance_below
        joint_constraint.weight = constraint_weight
        constraints.joint_constraints.append(joint_constraint)

    return constraints


def build_move_group_goal(
    target_joints: Sequence[float],
    *,
    planning_group: str = 'manipulator',
    vel: float = 0.1,
    acc: float = 0.1,
    plan_only: bool = False,
    joint_names: Sequence[str] = DEFAULT_JOINT_NAMES,
    tolerance_above: float = 0.01,
    tolerance_below: float = 0.01,
    constraint_weight: float = 1.0,
) -> MoveGroup.Goal:
    """Build a `MoveGroup.Goal` for a target joint vector in radians."""
    constraints = build_joint_constraints(
        target_joints,
        joint_names=joint_names,
        tolerance_above=tolerance_above,
        tolerance_below=tolerance_below,
        constraint_weight=constraint_weight,
    )

    goal_msg = MoveGroup.Goal()
    goal_msg.request = MotionPlanRequest()
    goal_msg.request.group_name = planning_group
    goal_msg.request.goal_constraints.append(constraints)
    goal_msg.request.max_velocity_scaling_factor = float(vel)
    goal_msg.request.max_acceleration_scaling_factor = float(acc)

    goal_msg.planning_options = PlanningOptions()
    goal_msg.planning_options.plan_only = plan_only
    return goal_msg


class JointMotionClient:
    """
    Reusable wrapper around the MoveIt `/move_action` client for joint goals.

    Create this from any existing `Node`, then call `send_goal()` or
    `send_goal_degrees()` with your target joint vector.
    """

    def __init__(
        self,
        node: Node,
        *,
        planning_group: str = 'manipulator',
        vel: float = 0.1,
        acc: float = 0.1,
        action_name: str = '/move_action',
        joint_names: Sequence[str] = DEFAULT_JOINT_NAMES,
        tolerance_above: float = 0.01,
        tolerance_below: float = 0.01,
        constraint_weight: float = 1.0,
        plan_only: bool = False,
    ):
        self.node = node
        self.planning_group = planning_group
        self.vel = float(vel)
        self.acc = float(acc)
        self.joint_names = tuple(joint_names)
        self.tolerance_above = float(tolerance_above)
        self.tolerance_below = float(tolerance_below)
        self.constraint_weight = float(constraint_weight)
        self.plan_only = bool(plan_only)
        self.action_client = ActionClient(node, MoveGroup, action_name)

    def wait_for_server(self, timeout_sec: Optional[float] = None) -> bool:
        """Wait for the MoveIt action server to become available."""
        if timeout_sec is None:
            self.action_client.wait_for_server()
            return True
        return self.action_client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal(
        self,
        target_joints: Sequence[float],
        *,
        goal_response_callback: Optional[Callable[[object], None]] = None,
        result_callback: Optional[Callable[[int, object], None]] = None,
    ):
        """Send a target joint vector in radians to MoveIt asynchronously."""
        goal_msg = build_move_group_goal(
            target_joints,
            planning_group=self.planning_group,
            vel=self.vel,
            acc=self.acc,
            plan_only=self.plan_only,
            joint_names=self.joint_names,
            tolerance_above=self.tolerance_above,
            tolerance_below=self.tolerance_below,
            constraint_weight=self.constraint_weight,
        )
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda done_future: self._handle_goal_response(
                done_future,
                goal_response_callback,
                result_callback,
            )
        )
        return future

    def send_goal_degrees(
        self,
        target_joints_deg: Sequence[float],
        *,
        goal_response_callback: Optional[Callable[[object], None]] = None,
        result_callback: Optional[Callable[[int, object], None]] = None,
    ):
        """Convert degrees to radians and send the result asynchronously."""
        return self.send_goal(
            degrees_to_radians(target_joints_deg),
            goal_response_callback=goal_response_callback,
            result_callback=result_callback,
        )

    def _handle_goal_response(
        self,
        future,
        goal_response_callback: Optional[Callable[[object], None]],
        result_callback: Optional[Callable[[int, object], None]],
    ):
        goal_handle = future.result()
        if goal_response_callback is not None:
            goal_response_callback(goal_handle)

        if not goal_handle.accepted:
            if result_callback is not None:
                result_callback(GOAL_REJECTED_ERROR_CODE, None)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future: self._handle_result(done_future, result_callback)
        )

    @staticmethod
    def _handle_result(future, result_callback: Optional[Callable[[int, object], None]]):
        wrapped_result = future.result()
        result = wrapped_result.result
        error_code = result.error_code.val
        if result_callback is not None:
            result_callback(error_code, result)


def _validate_joint_vector(
    joint_values: Sequence[float],
    joint_names: Sequence[str],
) -> None:
    if len(joint_values) != len(joint_names):
        raise ValueError(
            f'Expected {len(joint_names)} joint values for {tuple(joint_names)}, '
            f'got {len(joint_values)}.'
        )