"""Tests for modular speed-scaling helpers and wrappers."""

from types import SimpleNamespace

import pytest

from fanuc_tools.motion import modular_speed_demo
from fanuc_tools.motion import speed_control
from fanuc_tools.motion import speed_scaling


class FakePublisher:
    """Collects published Int32 data values."""

    def __init__(self):
        self.data = []

    def publish(self, msg):
        self.data.append(msg.data)


class FakeNode:
    """Minimal node stub for SpeedScalingClient tests."""

    def __init__(self):
        self.publisher = FakePublisher()

    def create_publisher(self, _msg_type, _topic, _qos_depth):
        return self.publisher


class FakeLogger:
    """Simple logger recorder used for non-ROS method tests."""

    def __init__(self):
        self.info_messages = []
        self.warn_messages = []
        self.error_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def warn(self, message):
        self.warn_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)


def test_clamp_speed_bounds_values():
    """clamp_speed should clamp values into [0, 100]."""
    assert speed_control.clamp_speed(-10.0) == pytest.approx(0.0)
    assert speed_control.clamp_speed(120.0) == pytest.approx(100.0)
    assert speed_control.clamp_speed(42.5) == pytest.approx(42.5)


def test_speed_to_int_rounds_and_clamps():
    """speed_to_int should round and clamp for Int32 publishing."""
    assert speed_control.speed_to_int(12.4) == 12
    assert speed_control.speed_to_int(12.6) == 13
    assert speed_control.speed_to_int(-5.0) == 0
    assert speed_control.speed_to_int(150.0) == 100


def test_speed_scaling_controller_operations():
    """Controller operations should update speed predictably."""
    controller = speed_control.SpeedScalingController(initial_speed=20.0)

    assert controller.get_speed() == pytest.approx(20.0)
    assert controller.increase(15.0) == pytest.approx(35.0)
    assert controller.decrease(10.0) == pytest.approx(25.0)
    assert controller.pause() == pytest.approx(0.0)
    assert controller.full_speed() == pytest.approx(100.0)
    assert controller.to_int() == 100


def test_speed_scaling_client_publishes_current_value():
    """SpeedScalingClient should publish Int32 values via node publisher."""
    node = FakeNode()
    client = speed_control.SpeedScalingClient(node, initial_speed=110.0)

    published = client.publish_current()

    assert published == 100
    assert node.publisher.data[-1] == 100


def test_speed_scaling_client_set_speed_with_publish():
    """set_speed(..., publish=True) should update state and publish immediately."""
    node = FakeNode()
    client = speed_control.SpeedScalingClient(node, initial_speed=0.0)

    new_speed = client.set_speed(33.6, publish=True)

    assert new_speed == pytest.approx(33.6)
    assert node.publisher.data[-1] == 34


def test_parse_speed_sequence_clamps_and_skips_empty_tokens():
    """Sequence parser should clamp values and ignore empty comma entries."""
    sequence = modular_speed_demo.parse_speed_sequence('100, 70, -5, 250, , 40')
    assert sequence == [100.0, 70.0, 0.0, 100.0, 40.0]


def test_speed_scaling_publish_speed_delegates_to_client():
    """SpeedScalingNode.publish_speed should delegate to speed client."""
    node = speed_scaling.SpeedScalingNode.__new__(speed_scaling.SpeedScalingNode)
    called = {'count': 0}

    class SpeedClientStub:
        def publish_current(self):
            called['count'] += 1

    node.speed_client = SpeedClientStub()
    speed_scaling.SpeedScalingNode.publish_speed(node)

    assert called['count'] == 1


def test_speed_scaling_read_input_sets_value_then_quits(monkeypatch):
    """read_input should accept numeric input and quit on 'q'."""
    node = speed_scaling.SpeedScalingNode.__new__(speed_scaling.SpeedScalingNode)
    logger = FakeLogger()
    set_calls = []
    shutdown_called = {'flag': False}

    class SpeedClientStub:
        def set_speed(self, value):
            set_calls.append(value)

    inputs = iter(['25', 'q'])

    node.speed_client = SpeedClientStub()
    node.get_logger = lambda: logger

    monkeypatch.setattr('builtins.input', lambda _prompt='': next(inputs))
    monkeypatch.setattr(speed_scaling.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(
        speed_scaling.rclpy,
        'shutdown',
        lambda: shutdown_called.__setitem__('flag', True),
    )

    speed_scaling.SpeedScalingNode.read_input(node)

    assert set_calls == [25.0]
    assert shutdown_called['flag'] is True


def test_speed_scaling_read_input_warns_for_out_of_range(monkeypatch):
    """Out-of-range values should warn and not call set_speed."""
    node = speed_scaling.SpeedScalingNode.__new__(speed_scaling.SpeedScalingNode)
    logger = FakeLogger()
    set_calls = []

    class SpeedClientStub:
        def set_speed(self, value):
            set_calls.append(value)

    inputs = iter(['120', 'q'])

    node.speed_client = SpeedClientStub()
    node.get_logger = lambda: logger

    monkeypatch.setattr('builtins.input', lambda _prompt='': next(inputs))
    monkeypatch.setattr(speed_scaling.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(speed_scaling.rclpy, 'shutdown', lambda: None)

    speed_scaling.SpeedScalingNode.read_input(node)

    assert set_calls == []
    assert any('out of range' in message for message in logger.warn_messages)


def test_modular_speed_demo_tick_publishes_each_step(monkeypatch):
    """Demo tick should publish sequence and shutdown when complete."""
    node = modular_speed_demo.ModularSpeedDemoNode.__new__(
        modular_speed_demo.ModularSpeedDemoNode
    )
    logger = FakeLogger()
    shutdown_called = {'flag': False}
    node.get_logger = lambda: logger

    published = []

    class SpeedClientStub:
        def set_speed(self, value, publish=False):
            published.append((value, publish))
            return value

        @property
        def controller(self):
            return SimpleNamespace(to_int=lambda: int(round(published[-1][0])))

    node.speed_client = SpeedClientStub()
    node.sequence = [100.0, 70.0]
    node.index = 0

    monkeypatch.setattr(
        modular_speed_demo.rclpy,
        'shutdown',
        lambda: shutdown_called.__setitem__('flag', True),
    )

    modular_speed_demo.ModularSpeedDemoNode.tick(node)
    modular_speed_demo.ModularSpeedDemoNode.tick(node)
    modular_speed_demo.ModularSpeedDemoNode.tick(node)

    assert published == [(100.0, True), (70.0, True)]
    assert shutdown_called['flag'] is True
