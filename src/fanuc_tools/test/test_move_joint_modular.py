"""Tests for modular joint-motion helpers and move_joint glue code."""

import math
from types import SimpleNamespace

import pytest

from fanuc_tools.motion import joint_motion
from fanuc_tools.motion import move_joint


class DummyParameter:
    """Small parameter wrapper matching rclpy parameter access pattern."""

    def __init__(self, value):
        self.value = value


class DummyNode:
    """Simple node stub for parameter declaration and retrieval tests."""

    def __init__(self, initial_params=None):
        self.params = dict(initial_params or {})

    def declare_parameter(self, name, default_value):
        self.params.setdefault(name, default_value)

    def get_parameter(self, name):
        return DummyParameter(self.params[name])


class ImmediateFuture:
    """Immediate future stub that executes callbacks synchronously."""

    def __init__(self, result):
        self._result = result

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._result


class FakeGoalHandle:
    """Fake goal handle for accepted/rejected action responses."""

    def __init__(self, accepted=True, error_code=1):
        self.accepted = accepted
        self._error_code = error_code

    def get_result_async(self):
        wrapped_result = SimpleNamespace(
            result=SimpleNamespace(
                error_code=SimpleNamespace(val=self._error_code)
            )
        )
        return ImmediateFuture(wrapped_result)


class FakeActionClient:
    """Action client stub that stores last sent goal."""

    def __init__(self, node, _action_type, action_name):
        self.node = node
        self.action_name = action_name
        self.goal_handle = FakeGoalHandle()
        self.last_goal = None
        self.wait_calls = []

    def wait_for_server(self, timeout_sec=None):
        self.wait_calls.append(timeout_sec)
        return True

    def send_goal_async(self, goal_msg):
        self.last_goal = goal_msg
        return ImmediateFuture(self.goal_handle)


class FakeLogger:
    """Logger stub used to inspect move_joint method side effects."""

    def __init__(self):
        self.info_messages = []
        self.error_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)


def make_fake_action_client_class(goal_handle):
    """Create a FakeActionClient class bound to a specific goal handle."""

    class BoundFakeActionClient(FakeActionClient):
        def __init__(self, node, action_type, action_name):
            super().__init__(node, action_type, action_name)
            self.goal_handle = goal_handle

    return BoundFakeActionClient


def make_partial_move_joint_node():
    """Create a partially initialized MoveJointNode for method-only tests."""
    node = move_joint.MoveJointNode.__new__(move_joint.MoveJointNode)
    node.config = SimpleNamespace(delay_between_moves=2.0, startup_delay=5.0)
    node.current_target = 'A'
    node.motion_in_progress = False
    node.iteration = 0
    node.start_time = 0.0
    node.next_move_time = 0.0
    node.startup_wait_logged = False
    node.position_a = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    node.position_b = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    return node


def test_declare_move_joint_parameters_declares_defaults():
    """declare_move_joint_parameters should expose all expected keys."""
    node = DummyNode()
    move_joint.declare_move_joint_parameters(node)

    assert node.params['planning_group'] == 'manipulator'
    assert node.params['vel'] == pytest.approx(0.1)
    assert node.params['acc'] == pytest.approx(0.1)
    assert node.params['delay_between_moves'] == pytest.approx(2.0)
    assert node.params['startup_delay'] == pytest.approx(5.0)

    assert len([k for k in node.params if k.startswith('position_a.')]) == 6
    assert len([k for k in node.params if k.startswith('position_b.')]) == 6


def test_read_joint_vector_degrees_reads_expected_order():
    """read_joint_vector_degrees should return joints in 1..6 order."""
    initial = {
        'position_a.joint_1': 10.0,
        'position_a.joint_2': 20.0,
        'position_a.joint_3': 30.0,
        'position_a.joint_4': 40.0,
        'position_a.joint_5': 50.0,
        'position_a.joint_6': 60.0,
    }
    node = DummyNode(initial)
    values = move_joint.read_joint_vector_degrees(node, 'position_a')
    assert values == [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]


def test_move_joint_loop_config_from_node_parameters_uses_values():
    """MoveJointLoopConfig should load custom parameter values correctly."""
    params = {
        'planning_group': 'custom_group',
        'vel': 0.5,
        'acc': 0.4,
        'startup_delay': 3.0,
        'delay_between_moves': 1.2,
        'position_a.joint_1': 1.0,
        'position_a.joint_2': 2.0,
        'position_a.joint_3': 3.0,
        'position_a.joint_4': 4.0,
        'position_a.joint_5': 5.0,
        'position_a.joint_6': 6.0,
        'position_b.joint_1': -1.0,
        'position_b.joint_2': -2.0,
        'position_b.joint_3': -3.0,
        'position_b.joint_4': -4.0,
        'position_b.joint_5': -5.0,
        'position_b.joint_6': -6.0,
    }
    node = DummyNode(params)

    config = move_joint.MoveJointLoopConfig.from_node_parameters(node)

    assert config.planning_group == 'custom_group'
    assert config.vel == pytest.approx(0.5)
    assert config.acc == pytest.approx(0.4)
    assert config.startup_delay == pytest.approx(3.0)
    assert config.delay_between_moves == pytest.approx(1.2)
    assert config.position_a_deg == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert config.position_b_deg == (-1.0, -2.0, -3.0, -4.0, -5.0, -6.0)


def test_degrees_radians_round_trip_conversion():
    """Degree/radian conversion helpers should preserve values."""
    values_deg = [30.0, -20.0, 35.0, 0.0, 10.0, 0.0]
    values_rad = joint_motion.degrees_to_radians(values_deg)

    assert values_rad[0] == pytest.approx(math.pi / 6.0)
    assert joint_motion.radians_to_degrees(values_rad) == pytest.approx(values_deg)


def test_named_joint_map_raises_on_length_mismatch():
    """named_joint_map should validate vector length."""
    with pytest.raises(ValueError):
        joint_motion.named_joint_map([1.0, 2.0])


def test_build_joint_constraints_populates_expected_fields():
    """build_joint_constraints should create one constraint per joint."""
    target = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

    constraints = joint_motion.build_joint_constraints(
        target,
        tolerance_above=0.02,
        tolerance_below=0.03,
        constraint_weight=0.75,
    )

    assert len(constraints.joint_constraints) == 6
    first = constraints.joint_constraints[0]
    assert first.joint_name == 'J1'
    assert first.position == pytest.approx(0.1)
    assert first.tolerance_above == pytest.approx(0.02)
    assert first.tolerance_below == pytest.approx(0.03)
    assert first.weight == pytest.approx(0.75)


def test_build_move_group_goal_populates_request_fields():
    """build_move_group_goal should carry planning and scaling options."""
    target = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]

    goal = joint_motion.build_move_group_goal(
        target,
        planning_group='manipulator',
        vel=0.6,
        acc=0.4,
        plan_only=True,
    )

    assert goal.request.group_name == 'manipulator'
    assert goal.request.max_velocity_scaling_factor == pytest.approx(0.6)
    assert goal.request.max_acceleration_scaling_factor == pytest.approx(0.4)
    assert goal.planning_options.plan_only is True
    assert len(goal.request.goal_constraints[0].joint_constraints) == 6


def test_joint_motion_client_rejected_goal_calls_result_callback(monkeypatch):
    """Rejected goals should emit GOAL_REJECTED_ERROR_CODE callback."""
    action_cls = make_fake_action_client_class(FakeGoalHandle(accepted=False))
    monkeypatch.setattr(joint_motion, 'ActionClient', action_cls)

    client = joint_motion.JointMotionClient(DummyNode())
    seen = {}

    def on_result(error_code, result):
        seen['error_code'] = error_code
        seen['result'] = result

    client.send_goal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], result_callback=on_result)

    assert seen['error_code'] == joint_motion.GOAL_REJECTED_ERROR_CODE
    assert seen['result'] is None


def test_joint_motion_client_accepted_goal_invokes_callbacks(monkeypatch):
    """Accepted goals should call response and result callbacks."""
    action_cls = make_fake_action_client_class(
        FakeGoalHandle(accepted=True, error_code=1)
    )
    monkeypatch.setattr(joint_motion, 'ActionClient', action_cls)

    client = joint_motion.JointMotionClient(DummyNode(), vel=0.5, acc=0.3)
    seen = {}

    def on_goal(goal_handle):
        seen['accepted'] = goal_handle.accepted

    def on_result(error_code, _result):
        seen['error_code'] = error_code

    client.send_goal_degrees(
        [0.0, 30.0, 0.0, 0.0, 0.0, 0.0],
        goal_response_callback=on_goal,
        result_callback=on_result,
    )

    assert seen['accepted'] is True
    assert seen['error_code'] == 1

    sent_goal = client.action_client.last_goal
    joint_2 = sent_goal.request.goal_constraints[0].joint_constraints[1]
    assert joint_2.position == pytest.approx(math.radians(30.0))


def test_move_joint_send_goal_uses_client_and_callbacks():
    """MoveJointNode.send_goal should delegate to JointMotionClient."""
    node = make_partial_move_joint_node()
    sent = {}

    class MotionClientStub:
        def wait_for_server(self):
            sent['waited'] = True

        def send_goal(self, target, goal_response_callback, result_callback):
            sent['target'] = target
            sent['goal_cb'] = goal_response_callback
            sent['result_cb'] = result_callback
            return 'future-token'

    node.motion_client = MotionClientStub()
    result = move_joint.MoveJointNode.send_goal(node, [0.1] * 6)

    assert sent['waited'] is True
    assert sent['target'] == [0.1] * 6
    assert sent['goal_cb'] == node.goal_response_callback
    assert sent['result_cb'] == node.result_callback
    assert result == 'future-token'


def test_move_joint_goal_response_callback_logs_when_accepted():
    """goal_response_callback should log only for accepted goals."""
    node = make_partial_move_joint_node()

    move_joint.MoveJointNode.goal_response_callback(
        node,
        SimpleNamespace(accepted=True),
    )
    assert any('Goal ACCEPTED' in msg for msg in node._logger.info_messages)


def test_move_joint_result_callback_updates_state(monkeypatch):
    """result_callback should flip target and release motion lock."""
    node = make_partial_move_joint_node()
    node.current_target = 'A'
    node.motion_in_progress = True
    monkeypatch.setattr(move_joint.time, 'monotonic', lambda: 100.0)

    move_joint.MoveJointNode.result_callback(node, 1, None)

    assert node.iteration == 1
    assert node.current_target == 'B'
    assert node.motion_in_progress is False
    assert node.next_move_time == pytest.approx(102.0)


def test_move_joint_loop_tick_waits_during_startup(monkeypatch):
    """loop_tick should not dispatch a goal before next_move_time."""
    node = make_partial_move_joint_node()
    node.start_time = 0.0
    node.next_move_time = 10.0
    node.motion_in_progress = False
    node.send_goal = lambda _target: pytest.fail('send_goal should not run')

    monkeypatch.setattr(move_joint.time, 'monotonic', lambda: 1.0)

    move_joint.MoveJointNode.loop_tick(node)

    assert node.startup_wait_logged is True


def test_move_joint_loop_tick_dispatches_when_ready(monkeypatch):
    """loop_tick should send current target when startup window has passed."""
    node = make_partial_move_joint_node()
    node.current_target = 'A'
    node.next_move_time = 10.0
    node.motion_in_progress = False
    dispatched = {}

    def capture_send_goal(target):
        dispatched['target'] = target

    node.send_goal = capture_send_goal
    monkeypatch.setattr(move_joint.time, 'monotonic', lambda: 11.0)

    move_joint.MoveJointNode.loop_tick(node)

    assert node.motion_in_progress is True
    assert dispatched['target'] == node.position_a
