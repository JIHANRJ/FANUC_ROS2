"""Live joint velocity plotter for FANUC robots.

Subscribes to /joint_states and plots per-joint velocity in real time.
If JointState.velocity is unavailable, velocities are estimated from
position deltas.
"""

from __future__ import annotations

import argparse
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


DEFAULT_JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']


class JointVelocityPlotter(Node):
    """Collects joint velocities and stores history for plotting."""

    def __init__(
        self,
        *,
        topic: str,
        joint_names: list[str],
        history_sec: float,
        use_msg_velocity: bool,
    ):
        super().__init__('joint_velocity_plotter')

        self.joint_names = list(joint_names)
        self.history_sec = float(history_sec)
        self.use_msg_velocity = bool(use_msg_velocity)

        self.max_samples = 4000
        self.time_buffer = deque(maxlen=self.max_samples)
        self.velocity_buffers = {
            name: deque(maxlen=self.max_samples) for name in self.joint_names
        }

        self._last_positions: dict[str, float] | None = None
        self._last_time: float | None = None
        self._lock = threading.Lock()

        self.create_subscription(JointState, topic, self._joint_state_callback, 50)
        self.get_logger().info(f'Listening on {topic} for joints: {self.joint_names}')

    def _stamp_to_seconds(self, msg: JointState) -> float:
        stamp = msg.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if stamp_sec > 0.0:
            return stamp_sec
        return time.time()

    def _joint_state_callback(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        now_sec = self._stamp_to_seconds(msg)
        name_to_pos = dict(zip(msg.name, msg.position))

        has_valid_velocity = (
            self.use_msg_velocity and len(msg.velocity) == len(msg.name) and len(msg.velocity) > 0
        )
        name_to_vel = dict(zip(msg.name, msg.velocity)) if has_valid_velocity else {}

        with self._lock:
            self.time_buffer.append(now_sec)

            dt = None
            if self._last_time is not None:
                dt = max(1e-6, now_sec - self._last_time)

            for joint in self.joint_names:
                if joint not in name_to_pos:
                    self.velocity_buffers[joint].append(0.0)
                    continue

                if joint in name_to_vel:
                    velocity = float(name_to_vel[joint])
                elif self._last_positions is not None and dt is not None and joint in self._last_positions:
                    velocity = (float(name_to_pos[joint]) - float(self._last_positions[joint])) / dt
                else:
                    velocity = 0.0

                self.velocity_buffers[joint].append(velocity)

            self._last_positions = name_to_pos
            self._last_time = now_sec

    def get_plot_data(self):
        """Return a snapshot of buffered data for plotting."""
        with self._lock:
            if not self.time_buffer:
                return None

            t_latest = self.time_buffer[-1]
            times = [t - t_latest for t in self.time_buffer]
            vel_map = {
                name: list(self.velocity_buffers[name])
                for name in self.joint_names
            }
            return times, vel_map


def spin_node(node: Node) -> None:
    rclpy.spin(node)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Plot live FANUC joint velocities.')
    parser.add_argument('--topic', default='/joint_states', help='JointState topic name.')
    parser.add_argument(
        '--joints',
        default=','.join(DEFAULT_JOINT_NAMES),
        help='Comma-separated joint names to plot, e.g. J1,J2,J3,J4,J5,J6',
    )
    parser.add_argument('--history-sec', type=float, default=10.0, help='Time history window.')
    parser.add_argument(
        '--no-msg-velocity',
        action='store_true',
        help='Ignore JointState.velocity and always estimate from position deltas.',
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    joint_names = [name.strip() for name in args.joints.split(',') if name.strip()]
    if not joint_names:
        raise ValueError('No joint names provided.')

    rclpy.init()
    node = JointVelocityPlotter(
        topic=args.topic,
        joint_names=joint_names,
        history_sec=args.history_sec,
        use_msg_velocity=not args.no_msg_velocity,
    )

    spinner = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spinner.start()

    cols = 3
    rows = (len(joint_names) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(14, 4 * rows), sharex=True)
    fig.suptitle('Live Joint Velocities (rad/s)')

    if hasattr(axes, 'flatten'):
        axes = list(axes.flatten())
    else:
        axes = [axes]

    line_map = {}
    for index, joint in enumerate(joint_names):
        ax = axes[index]
        line, = ax.plot([], [], linewidth=1.8)
        line_map[joint] = (ax, line)
        ax.set_title(joint)
        ax.set_ylabel('rad/s')
        ax.grid(True, alpha=0.3)

    for ax in axes[len(joint_names):]:
        ax.axis('off')

    axes[min(len(axes) - 1, len(joint_names) - 1)].set_xlabel('time (s, latest at 0)')

    def update(_frame_index):
        snapshot = node.get_plot_data()
        if snapshot is None:
            return []

        times, velocities = snapshot
        t_min = -abs(args.history_sec)
        t_max = 0.0

        artists = []
        for joint in joint_names:
            ax, line = line_map[joint]
            joint_vel = velocities.get(joint, [])
            if not joint_vel:
                continue

            line.set_data(times, joint_vel)
            ax.set_xlim(t_min, t_max)

            v_min = min(joint_vel)
            v_max = max(joint_vel)
            if abs(v_max - v_min) < 1e-6:
                pad = max(0.05, abs(v_max) * 0.2 + 0.05)
            else:
                pad = (v_max - v_min) * 0.15
            ax.set_ylim(v_min - pad, v_max + pad)
            artists.append(line)

        return artists

    anim = FuncAnimation(fig, update, interval=120, blit=False)

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        del anim
        node.destroy_node()
        rclpy.shutdown()
        spinner.join(timeout=2.0)


if __name__ == '__main__':
    main()
