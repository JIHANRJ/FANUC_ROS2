"""Tests for modular cartesian motion helpers and move_cartesian glue code."""

import math
from types import SimpleNamespace

import pytest

from fanuc_tools.motion import cartesian_motion
from fanuc_tools.motion import move_cartesian


class DummyParameter:
    """Small parameter wrapper matching rclpy parameter access pattern."""

    def __init__(self, value):
        self.value = value


class DummyNode:
    """Simple node stub for parameter declaration and retrieval tests."""

    def __init__(self, initial_params=None):
        self.params = dict(initial_params or {})
        self.subscriptions = []
        self.clients = []

    def declare_parameter(self, name, default_value):
        self.params.setdefault(name, default_value)

    def get_parameter(self, name):
        return DummyParameter(self.params[name])

    def create_subscription(self, *args, **kwargs):
        self.subscriptions.append((args, kwargs))
        return object()

    def create_client(self, srv_type, name):
        client = FakeServiceClient()
        self.clients.append((srv_type, name, client))
        return client

    def get_logger(self):
        return FakeLogger()


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
            result=SimpleNamespace(error_code=SimpleNamespace(val=self._error_code))
        )
        return ImmediateFuture(wrapped_result)


class FakeActionClient:
    """Action client stub that stores the last sent goal."""

    def __init__(self, node, _action_type, action_name):
        self.node = node
        self.action_name = action_name
        self.goal_handle = FakeGoalHandle()
        self.last_goal = None

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal_msg):
        self.last_goal = goal_msg
        return ImmediateFuture(self.goal_handle)


class FakeServiceClient:
    """Service client stub used for cartesian path planning."""

    def __init__(self):
        self.request = None
        self.response = SimpleNamespace(
            error_code=SimpleNamespace(val=1),
            fraction=1.0,
            solution=SimpleNamespace(
                joint_trajectory=SimpleNamespace(
                    joint_names=['J1', 'J2'],
                    points=[],
                )
            ),
        )

    def wait_for_service(self, timeout_sec=None):
        return True

    def call_async(self, request):
        self.request = request
        return ImmediateFuture(self.response)


class FakeLogger:
    """Logger stub used to inspect node side effects."""

    def __init__(self):
        self.info_messages = []
        self.error_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)


def make_partial_move_cartesian_node():
    """Create a partially initialized MoveCartesianNode for method-only tests."""
    node = move_cartesian.MoveCartesianNode.__new__(move_cartesian.MoveCartesianNode)
    node.config = SimpleNamespace(delay_between_moves=2.0)
    node.current_target = 'A'
    node.iteration = 0
    node.motion_in_progress = True
    node.next_attempt_time = 0.0
    node.active_index = 0
    node.pending_next_index = 0
    node.active_sequence = tuple()
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    return node


def test_rpy_quaternion_round_trip_conversion():
    """Quaternion helpers should preserve the original orientation."""
    qx, qy, qz, qw = cartesian_motion.rpy_to_quaternion(0.2, -0.1, 0.4)
    roll, pitch, yaw = cartesian_motion.quaternion_to_rpy(qx, qy, qz, qw)

    assert roll == pytest.approx(0.2)
    assert pitch == pytest.approx(-0.1)
    assert yaw == pytest.approx(0.4)


def test_slerp_quaternion_midpoint_rotates_halfway():
    """SLERP should produce a midpoint orientation."""
    q1 = cartesian_motion.rpy_to_quaternion(0.0, 0.0, 0.0)
    q2 = cartesian_motion.rpy_to_quaternion(0.0, 0.0, math.pi / 2.0)

    midpoint = cartesian_motion.slerp_quaternion(q1, q2, 0.5)
    _, _, yaw = cartesian_motion.quaternion_to_rpy(*midpoint)

    assert yaw == pytest.approx(math.pi / 4.0, abs=1e-5)


def test_validate_waypoint_spacing_rejects_close_points():
    """Waypoints closer than 1 mm should be rejected."""
    waypoints = [
        cartesian_motion.CartesianWaypoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        cartesian_motion.CartesianWaypoint(0.0005, 0.0, 0.0, 0.0, 0.0, 0.0),
    ]

    is_valid, reason = cartesian_motion.validate_waypoint_spacing(waypoints)

    assert not is_valid
    assert 'apart' in reason


def test_interpolate_cartesian_segment_adds_intermediate_points():
    """Large moves should be densified for smoother CNT planning."""
    start = cartesian_motion.CartesianWaypoint(0.0, 0.0, 0.0, math.pi, 0.0, 0.0)
    end = cartesian_motion.CartesianWaypoint(0.10, 0.0, 0.0, math.pi, 0.0, 0.0)

    poses = cartesian_motion.interpolate_cartesian_segment(
        start,
        end,
        max_position_step_m=0.02,
    )

    assert len(poses) == 6
    assert poses[-1].position.x == pytest.approx(0.10)


def test_build_move_group_goal_populates_pose_constraints():
    """FINE goals should carry one pose constraint set."""
    waypoint = cartesian_motion.CartesianWaypoint(0.5, 0.0, 0.4, math.pi, 0.0, 0.0)
    goal = cartesian_motion.build_move_group_goal(waypoint, planning_group='manipulator')

    assert goal.request.group_name == 'manipulator'
    assert len(goal.request.goal_constraints) == 1


def test_parse_waypoint_sequence_json_reads_cnt_and_fine_entries():
    """JSON sequence parsing should load both FINE and CNT waypoints."""
    sequence = move_cartesian.parse_waypoint_sequence_json(
        '[{"x":0.5,"y":0.0,"z":0.5,"roll_deg":180,"pitch_deg":0,'
        '"yaw_deg":0,"velocity_mm_s":500,"blend_type":"FINE"},'
        '{"x":0.5,"y":0.2,"z":0.4,"roll_deg":180,"pitch_deg":0,'
        '"yaw_deg":0,"velocity_mm_s":400,"blend_type":"CNT","blend_radius_mm":100}]'
    )

    assert len(sequence) == 2
    assert sequence[0].normalized_blend_type() == 'FINE'
    assert sequence[1].normalized_blend_type() == 'CNT'
    assert sequence[1].blend_radius_m == pytest.approx(0.1)


def test_load_waypoint_sequence_falls_back_to_legacy_pose_params():
    """Legacy pose parameters should still produce one FINE waypoint."""
    node = DummyNode(
        {
            'position_a_sequence_json': '',
            'position_a.x': 0.5,
            'position_a.y': 0.1,
            'position_a.z': 0.2,
            'position_a.roll': 180.0,
            'position_a.pitch': 0.0,
            'position_a.yaw': 10.0,
            'nominal_speed_mm_s': 500.0,
        }
    )

    waypoint_list = move_cartesian.load_waypoint_sequence(node, 'position_a')

    assert len(waypoint_list) == 1
    assert waypoint_list[0].x == pytest.approx(0.5)
    assert waypoint_list[0].yaw_rad == pytest.approx(math.radians(10.0))


def test_cartesian_motion_client_send_goal_calls_callbacks(monkeypatch):
    """FINE motions should call goal and result callbacks on success."""
    monkeypatch.setattr(cartesian_motion, 'ActionClient', FakeActionClient)

    node = DummyNode()
    accepted = []
    results = []
    client = cartesian_motion.CartesianMotionClient(node)

    waypoint = cartesian_motion.CartesianWaypoint(0.5, 0.0, 0.5, math.pi, 0.0, 0.0)
    client.send_goal(
        waypoint,
        goal_response_callback=lambda goal_handle: accepted.append(goal_handle.accepted),
        result_callback=lambda error_code, _result: results.append(error_code),
    )

    assert accepted == [True]
    assert results == [1]


def test_cartesian_motion_client_send_path_rejects_invalid_spacing(monkeypatch):
    """Invalid CNT spacing should fail before planning service call."""
    monkeypatch.setattr(cartesian_motion, 'ActionClient', FakeActionClient)

    node = DummyNode()
    client = cartesian_motion.CartesianMotionClient(node)
    results = []
    invalid = [
        cartesian_motion.CartesianWaypoint(0.0, 0.0, 0.0, math.pi, 0.0, 0.0, blend_type='CNT'),
        cartesian_motion.CartesianWaypoint(0.0005, 0.0, 0.0, math.pi, 0.0, 0.0, blend_type='CNT'),
    ]

    client.send_path(invalid, result_callback=lambda error_code, _result: results.append(error_code))

    assert results == [cartesian_motion.GOAL_REJECTED_ERROR_CODE]


def test_move_cartesian_result_callback_advances_sequence():
    """Successful result callback should continue the sequence."""
    node = make_partial_move_cartesian_node()
    dispatched = []
    node.pending_next_index = 2
    node._dispatch_next_chunk = lambda: dispatched.append('next')

    move_cartesian.MoveCartesianNode.result_callback(node, 1, None)

    assert node.active_index == 2
    assert dispatched == ['next']


def test_move_cartesian_finish_sequence_flips_target_on_success():
    """Successful sequence completion should flip A to B."""
    node = make_partial_move_cartesian_node()

    move_cartesian.MoveCartesianNode._finish_sequence(node, success=True)

    assert node.current_target == 'B'
    assert not node.motion_in_progress
