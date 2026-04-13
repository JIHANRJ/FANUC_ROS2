"""
read_joint_state.py
===================
Motion module for reading and managing current joint states from the robot.

This module provides a reusable JointStateReader class that subscribes to
/joint_states topic and exposes the current joint positions for motion planning
and control workflows.

Module Classes:
    JointStateReader: Subscribes to /joint_states, stores latest joint positions,
                     provides blocking and non-blocking access methods.

Module Functions:
    main(): Standalone runner demonstrating real-time joint state monitoring.

Use Cases:
    1. **Motion Planning Seeding**: Get current joints to seed IK solvers
    2. **State Verification**: Check robot is at expected position before motion
    3. **Diagnostic Monitoring**: Watch joint angles in real-time with colored output
    4. **Motion Blending**: Read current pose to blend into new trajectory

Typical Usage (in motion scripts):
    >>> reader = JointStateReader()
    >>> rclpy.spin_once(node)  # pull one batch of messages
    >>> if reader.is_ready():
    ...     joints = reader.get_joints()  # {'J1': 0.12, 'J2': -0.45, ...}
    ...     j1_rad = reader.get_joint('J1')  # 0.12

Standalone Usage (monitoring):
    Terminal 1: ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \\
                    robot_model:=crx10ia_l use_mock:=true
    Terminal 2: python3 -m fanuc_tools.motion.nodes.read_joint_state

ROS Topics:
    /joint_states (sensor_msgs.msg.JointState):
        Publishes (typically by robot_state_publisher or joint_state_publisher)
        Contains: position (list[float] in radians), name, velocity, effort
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateReader(Node):
    """
    Subscribes to /joint_states and caches the latest joint positions.
    
    Thread-safe access to current robot joint configuration. All get_* methods
    are safe to call from timers, action callbacks, and service handlers.
    
    Attributes:
        JOINT_NAMES (list[str]): Expected joints for CRX-10iA/L.
                                 Set to ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'].
        _joint_positions (dict): Internal storage {'J1': 0.0, 'J2': -0.45, ...}
                                 Updated on every /joint_states message.
        _received (bool): True after first message; False otherwise.
                         Use is_ready() to check before accessing get_*.
    
    Example:
        reader = JointStateReader()
        rclpy.spin_once(reader, timeout_sec=1.0)  # wait for one batch
        
        if reader.is_ready():
            joints = reader.get_joints()  # dict
            j1 = reader.get_joint('J1')   # float or None
    """

    # Joint names for CRX-10iA/L
    JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']

    def __init__(self):
        """
        Initialize the JointStateReader node.
        
        Creates:
            - ROS 2 node named 'joint_state_reader'
            - Subscription to /joint_states topic (sensor_msgs.msg.JointState)
            - Internal storage dict for joint positions
        
        The node is ready to subscr but will not have valid data until
        the first message arrives. Use is_ready() to check.
        """
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

        self.get_logger().info('JointStateReader ready. Waiting for /joint_states...')

    # ── Private callback — not meant to be called directly ────────────────

    def _callback(self, msg):
        """
        Called automatically by ROS 2 when a JointState message arrives.
        
        Updates internal storage — does NOT print anything.
        Printing is handled separately by print_joints().
        
        Args:
            msg (sensor_msgs.msg.JointState): The message from /joint_states.
                Contains fields: name (list[str]), position (list[float]),
                velocity, effort, header.
        """
        for name, position in zip(msg.name, msg.position):
            if name in self._joint_positions:
                self._joint_positions[name] = position
        self._received = True

    # ── Public API — call these from anywhere ─────────────────────────────

    def get_joints(self):
        """
        Returns the latest joint positions as a dictionary.
        
        Safe to call from any context (timer, callback, service handler).
        
        Returns:
            dict: {'J1': float, 'J2': float, ...} with radians as values.
                  Returns None if no message received yet.
        
        Raises:
            None (returns None on error instead of raising)

        Example:
            joints = reader.get_joints()
            if joints:
                print(f"J1 = {joints['J1']:.4f} rad")
                print(f"All positions: {joints}")
        """
        if not self._received:
            return None
        return dict(self._joint_positions)

    def get_joint(self, name):
        """
        Returns the current position of a single joint in radians.
        
        Safe to call from any context. Useful for checking a specific axis
        before motion commands or for diagnostic logging.
        
        Args:
            name (str): Joint name e.g. 'J1', 'J2', 'J3', 'J4', 'J5', 'J6'.

        Returns:
            float: Position in radians (e.g., 0.0, -0.45, 1.57).
            None:  If not received yet or name not found in expected joints.

        Example:
            pos = reader.get_joint('J1')
            if pos is not None:
                print(f"J1 is at {pos:.4f} rad")
            else:
                print("J1 data not available yet")
        """
        if not self._received:
            return None
        return self._joint_positions.get(name)

    def is_ready(self):
        """
        Returns True if at least one /joint_states message has been received.
        
        Always check this before calling get_joints() or get_joint().
        Prevents returning stale/default values if subscription hasn't fired yet.
        
        Returns:
            bool: True if at least one message received and cached.
                  False if node is still waiting for first message.

        Example:
            if reader.is_ready():
                joints = reader.get_joints()
            else:
                print("Waiting for /joint_states to publish...")
        """
        return self._received

    def print_joints(self):
        """
        Prints current joint states neatly in-place (no scrolling).
        
        Creates a formatted ASCII table with columns for joint name, radians,
        and degrees. Colors J1-J6 green if |angle| < 5°, yellow otherwise.
        Uses ANSI escape codes to overwrite previous output (terminal only).
        
        Intended for standalone monitoring or diagnostic output.
        Not recommended for production logging (use get_joints() instead).
        
        Output Format:
            ┌────────┬──────────────┬──────────────┐
            │ Joint  │     Radians  │     Degrees  │
            ├────────┼──────────────┼──────────────┤
            │ J1     │     +0.0000  │     +0.00 °  │
            │ J2     │     -0.4500  │    -25.78 °  │
            ...
            └────────┴──────────────┴──────────────┘
        
        Call Frequency:
            - Typically attached to a ROS 2 timer at 10 Hz to avoid flicker
            - Direct calls in tight loops will produce flickering output
        
        Example:
            # In a loop (not recommended):
            # while rclpy.ok():
            #     rclpy.spin_once(reader)
            #     reader.print_joints()
            
            # Recommended:
            reader.create_timer(0.1, reader.print_joints)  # print at 10 Hz
            rclpy.spin(reader)
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
            deg_str = f'{deg:+.2f}°'       # e.g. '+0.00°'

            # Then apply colour to name only — not to the numbers
            # This keeps column widths predictable
            print(
                f'\033[K│ {colour}{name}{reset}     │ '
                f'{rad_str:>12} │ '
                f'{deg_str:>13} │'
            )

        print('\033[K└────────┴──────────────┴──────────────┘')




# ── Standalone runner ─────────────────────────────────────────────────────────

def main():
    """
    Standalone entry point for real-time joint state monitoring.
    
    Creates a JointStateReader node and prints joint positions at 10 Hz
    with a live-updating (non-scrolling) display.
    
    Usage:
        Terminal 1: ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \\
                        robot_model:=crx10ia_l use_mock:=true
        Terminal 2: python3 -m fanuc_tools.motion.nodes.read_joint_state
    
    Keyboard:
        Press Ctrl+C to exit cleanly.
    """
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
