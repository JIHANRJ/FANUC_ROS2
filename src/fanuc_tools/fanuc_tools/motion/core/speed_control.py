"""
speed_control.py
================
Reusable helpers for speed scaling control.

This module exposes simple, importable primitives that can be reused by
standalone nodes and other scripts.
"""

from __future__ import annotations

from std_msgs.msg import Int32


SCALING_TOPIC = '/speed_scaling_factor'
MIN_SPEED = 0.0
MAX_SPEED = 100.0


def clamp_speed(value: float, min_speed: float = MIN_SPEED, max_speed: float = MAX_SPEED) -> float:
    """Clamp speed to the inclusive range `[min_speed, max_speed]`."""
    return max(float(min_speed), min(float(max_speed), float(value)))


def speed_to_int(value: float) -> int:
    """Convert speed value to clamped Int32-compatible integer."""
    return int(round(clamp_speed(value)))


class SpeedScalingController:
    """Pure-python speed state and operations (no ROS dependency)."""

    def __init__(self, initial_speed: float = 100.0):
        self.speed = clamp_speed(initial_speed)

    def set_speed(self, value: float) -> float:
        """Set speed and return clamped value."""
        self.speed = clamp_speed(value)
        return self.speed

    def increase(self, step: float = 5.0) -> float:
        """Increase speed by `step` and return clamped value."""
        return self.set_speed(self.speed + float(step))

    def decrease(self, step: float = 5.0) -> float:
        """Decrease speed by `step` and return clamped value."""
        return self.set_speed(self.speed - float(step))

    def pause(self) -> float:
        """Set speed to 0 and return it."""
        return self.set_speed(0.0)

    def full_speed(self) -> float:
        """Set speed to 100 and return it."""
        return self.set_speed(100.0)

    def get_speed(self) -> float:
        """Get current speed as float."""
        return self.speed

    def to_int(self) -> int:
        """Get current speed as clamped integer for Int32 message data."""
        return speed_to_int(self.speed)


class SpeedScalingClient:
    """Small ROS helper that publishes speed scaling Int32 messages."""

    def __init__(
        self,
        node,
        *,
        topic: str = SCALING_TOPIC,
        initial_speed: float = 100.0,
        qos_depth: int = 10,
    ):
        self.node = node
        self.topic = topic
        self.controller = SpeedScalingController(initial_speed)
        self.publisher = node.create_publisher(Int32, topic, qos_depth)

    def set_speed(self, value: float, *, publish: bool = False) -> float:
        """Set speed and optionally publish immediately."""
        speed = self.controller.set_speed(value)
        if publish:
            self.publish_current()
        return speed

    def publish_current(self) -> int:
        """Publish current speed and return published integer value."""
        msg = Int32()
        msg.data = self.controller.to_int()
        self.publisher.publish(msg)
        return msg.data
