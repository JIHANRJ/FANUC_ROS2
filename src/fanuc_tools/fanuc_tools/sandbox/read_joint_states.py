"""
read_joint_states.py
====================
Sandbox script — learn how to read and display joint states.

Demonstrates:
    - How to subscribe to a topic
    - How to return data from a callback for use elsewhere
    - How to print neatly in-place without scrolling

Run:
    Terminal 1: ros2 launch fanuc_moveit_config fanuc_moveit.launch.py
                    robot_model:=crx10ia_l use_mock:=true
    Terminal 2: python3 sandbox/read_joint_states.py
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateReader(Node):
    """
    Subscribes to /joint_states and stores the latest values.

    Usage from another script:
        reader = JointStateReader()
        joints = reader.get_joints()   # returns dict
        pos    = reader.get_joint('J1')  # returns single float
    """

    # Joint names for CRX-10iA/L
    JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']

    def __init__(self):
        super().__init__('joint_state_reader')

        # Internal storage — updated every time a message arrives
        # Stored as a dict: {'J1': 0.0, 'J2': -0.45, ...}
        self._joint_positions = {name: 0.0 for name in self.JOINT_NAMES}
        self._received = False   # False until first message arrives

        # Subscriber — calls _callback every time /joint_states publishes
        self.create_subscription(
            JointState,
            '/joint_states',
            self._callback,
            10
        )

        self.get_logger().info('JointStateReader ready.')

    # ── Private callback — not meant to be called directly ────────────────

    def _callback(self, msg):
        """
        Called automatically by ROS 2 when a JointState message arrives.
        Updates internal storage — does NOT print anything.
        Printing is handled separately by print_joints().
        """
        for name, position in zip(msg.name, msg.position):
            if name in self._joint_positions:
                self._joint_positions[name] = position
        self._received = True

    # ── Public API — call these from anywhere ─────────────────────────────

    def get_joints(self):
        """
        Returns the latest joint positions as a dictionary.

        Returns:
            dict: {'J1': 0.12, 'J2': -0.45, 'J3': 1.23, ...}
            None: if no message received yet

        Example:
            joints = reader.get_joints()
            if joints:
                print(joints['J1'])
        """
        if not self._received:
            return None
        return dict(self._joint_positions)

    def get_joint(self, name):
        """
        Returns the current position of a single joint in radians.

        Args:
            name: str — joint name e.g. 'J1', 'J2'

        Returns:
            float: position in radians
            None:  if not received yet or name not found

        Example:
            pos = reader.get_joint('J1')
        """
        if not self._received:
            return None
        return self._joint_positions.get(name)

    def is_ready(self):
        """
        Returns True if at least one message has been received.
        Always check this before calling get_joints().

        Example:
            if reader.is_ready():
                joints = reader.get_joints()
        """
        return self._received

    def print_joints(self):
        """
        Prints current joint states neatly in-place.
        Overwrites previous output — no scrolling.
        """
        joints = self.get_joints()

        if joints is None:
            print('Waiting for /joint_states...')
            return

        NUM_LINES = len(self.JOINT_NAMES) + 4

        if hasattr(self, '_printed_once'):
            print(f'\033[{NUM_LINES}F', end='')
        else:
            self._printed_once = True

        # ── Fixed width columns ───────────────────────────────────────────
        # Colour codes are applied AFTER padding so they don't affect spacing
        print('\033[K┌────────┬──────────────┬──────────────┐')
        print('\033[K│ Joint  │     Radians  │     Degrees  │')
        print('\033[K├────────┼──────────────┼──────────────┤')

        for name, pos in joints.items():
            deg    = pos * 57.2958
            colour = '\033[92m' if abs(deg) < 5 else '\033[93m'
            reset  = '\033[0m'

            # Format numbers first with fixed width
            rad_str = f'{pos:+.4f} rad'    # e.g. '+0.0000 rad'
            deg_str = f'{deg:+.2f} deg'    # e.g. '+0.00 deg'

            # Then apply colour to name only — not to the numbers
            # This keeps column widths predictable
            print(
                f'\033[K│ {colour}{name}{reset}     │ '
                f'{rad_str:>12} │ '
                f'{deg_str:>12} │'
            )

        print('\033[K└────────┴──────────────┴──────────────┘')




# ── Standalone runner ─────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = JointStateReader()

    # Use a timer to print at a fixed rate (not on every message)
    # This prevents flickering from high-frequency updates
    node.create_timer(0.1, node.print_joints)  # print at 10Hz

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nStopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


