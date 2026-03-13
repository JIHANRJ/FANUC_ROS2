"""
jog_ps4.py
==========
Jog the FANUC robot joint-by-joint using a PlayStation 4 (DualShock 4)
controller via pygame.

Prerequisites
-------------
Install pygame (once):
    sudo apt install python3-pygame

Build & run:
    cd ~/ws_fanuc
    colcon build --symlink-install --packages-select fanuc_tools
    source install/setup.bash

    # Terminal 1 – MoveIt (simulation)
    ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \\
        robot_model:=crx10ia_l use_mock:=true

    # Terminal 2 – PS4 jog
    ros2 run fanuc_tools jog_ps4

    # Real robot: add  robot_ip:=<IP>  use_mock:=false to Terminal 1

Controller layout (DualShock 4)
--------------------------------
  Left  stick  X   →  Jog  J1  (base sweep)
  Left  stick  Y   →  Jog  J2  (shoulder)
  Right stick  Y   →  Jog  J3  (elbow)
  Right stick  X   →  Jog  J4  (forearm roll)
  L2  (trigger)    →  Jog  J5  negative  (wrist pitch ↓)
  R2  (trigger)    →  Jog  J5  positive  (wrist pitch ↑)
  L1  (bumper)     →  Jog  J6  negative  (tool roll ↓)
  R1  (bumper)     →  Jog  J6  positive  (tool roll ↑)
  D-pad  UP        →  Increase step size
  D-pad  DOWN      →  Decrease step size
  Cross   (×)      →  Go to home  [0, 0, 0, 0, 0, 0]
  Options  (☰)     →  Quit

Notes
-----
- Move step default: 0.05 rad per tick (~2.9 °)
- A new goal is only sent after the previous one completes (non-blocking).
- Joint limits are NOT enforced here; MoveIt will reject out-of-range targets.
- Tune DEADZONE / STEP_MIN / STEP_MAX / RATE_HZ to preference.
"""

import os
import sys
import math

# ── Silence SDL "no display" warning in headless / SSH environments ──────────
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')

try:
    import pygame
except ImportError:
    print("\n[jog_ps4] ERROR: pygame is not installed.")
    print("  Fix:  sudo apt install python3-pygame\n")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    RobotState,
)

# ── Tuning constants ─────────────────────────────────────────────────────────
DEADZONE   = 0.15      # ignore stick values below this magnitude
STEP_MIN   = 0.01      # rad  – smallest jog step  (~0.57 °)
STEP_MAX   = 0.20      # rad  – largest  jog step  (~11.5 °)
STEP_INIT  = 0.05      # rad  – default  jog step  (~2.9  °)
STEP_INC   = 0.01      # rad change per D-pad press
RATE_HZ    = 10.0      # jog tick rate
VEL        = 0.15      # MoveIt velocity scaling (0–1)
ACC        = 0.15      # MoveIt acceleration scaling (0–1)

JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]
HOME        = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# ── DS4 axis / button indices (standard pygame / Linux hid-sony mapping) ─────
AXIS_J1    = 0   # Left  stick X
AXIS_J2    = 1   # Left  stick Y  (positive = down  → inverted below)
AXIS_J3    = 3   # Right stick Y  (positive = down  → inverted below)
AXIS_J4    = 2   # Right stick X
AXIS_L2    = 4   # L2 trigger    (-1 = idle,  +1 = full press)
AXIS_R2    = 5   # R2 trigger    (-1 = idle,  +1 = full press)

BTN_L1     = 4
BTN_R1     = 5
BTN_CROSS  = 0   # ×  – go to home
BTN_OPTIONS= 9   # ☰ – quit

HAT_DPAD   = 0   # hat index; value is (x, y)


# ─────────────────────────────────────────────────────────────────────────────
class PS4JogNode(Node):
    """ROS 2 node that reads a DS4 controller and jogs the robot."""

    def __init__(self):
        super().__init__('ps4_jog_node')

        # ── Current joint state ──────────────────────────────────────────────
        self._current_joints: list[float] = list(HOME)
        self.create_subscription(JointState, '/joint_states',
                                 self._joint_state_cb, 10)

        # ── MoveIt action client ─────────────────────────────────────────────
        self._client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for /move_action server …')
        self._client.wait_for_server()
        self.get_logger().info('/move_action server found.')

        # ── State flags ──────────────────────────────────────────────────────
        self._moving   = False   # True while a goal is in-flight
        self._step     = STEP_INIT
        self._hat_prev = (0, 0)  # track D-pad edges

        # ── pygame init ──────────────────────────────────────────────────────
        pygame.init()
        pygame.joystick.init()
        self._joystick = self._find_joystick()

        # ── Jog timer ────────────────────────────────────────────────────────
        period = 1.0 / RATE_HZ
        self.create_timer(period, self._jog_tick)

        self.get_logger().info(
            f'\n'
            f'  PS4 Jog ready   (step = {math.degrees(self._step):.1f} °)\n'
            f'  Left  stick    → J1, J2\n'
            f'  Right stick    → J4, J3\n'
            f'  L2 / R2        → J5\n'
            f'  L1 / R1        → J6\n'
            f'  D-pad ↑/↓     → step ± {math.degrees(STEP_INC):.1f} °\n'
            f'  × (Cross)      → go home\n'
            f'  Options (☰)    → quit\n'
        )

    # ── Joystick discovery ───────────────────────────────────────────────────
    def _find_joystick(self):
        n = pygame.joystick.get_count()
        if n == 0:
            self.get_logger().error(
                'No joystick detected by pygame.  '
                'Make sure the PS4 controller is connected (USB or BT) '
                'and visible in /dev/input/js*')
            rclpy.shutdown()
            sys.exit(1)

        # Prefer a device with "sony", "wireless", "dualshock", or "gamepad"
        # in its name; fall back to the first one.
        chosen = None
        for i in range(n):
            js = pygame.joystick.Joystick(i)
            js.init()
            name_lower = js.get_name().lower()
            self.get_logger().info(f'  joystick {i}: "{js.get_name()}"')
            if chosen is None and any(k in name_lower for k in
                                      ('sony', 'wireless', 'dualshock',
                                       'gamepad', 'ps4', 'ps3', 'ps5')):
                chosen = js
        if chosen is None:
            chosen = pygame.joystick.Joystick(0)
            chosen.init()

        self.get_logger().info(f'Using joystick: "{chosen.get_name()}"')
        return chosen

    # ── Joint state callback ─────────────────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        mapping = dict(zip(msg.name, msg.position))
        for i, name in enumerate(JOINT_NAMES):
            if name in mapping:
                self._current_joints[i] = mapping[name]

    # ── Main jog tick (runs at RATE_HZ) ─────────────────────────────────────
    def _jog_tick(self):
        pygame.event.pump()

        js = self._joystick

        # ── Helper: safe axis read with deadzone ─────────────────────────────
        def axis(idx):
            try:
                v = js.get_axis(idx)
            except Exception:
                return 0.0
            return v if abs(v) > DEADZONE else 0.0

        def btn(idx):
            try:
                return js.get_button(idx)
            except Exception:
                return 0

        # ── Options → quit ───────────────────────────────────────────────────
        if btn(BTN_OPTIONS):
            self.get_logger().info('Options pressed – shutting down.')
            rclpy.shutdown()
            return

        # ── D-pad edge detection → step size ─────────────────────────────────
        hat = js.get_hat(HAT_DPAD) if js.get_numhats() > 0 else (0, 0)
        if hat != self._hat_prev:
            if hat[1] == 1 and self._hat_prev[1] != 1:   # D-pad UP
                self._step = min(self._step + STEP_INC, STEP_MAX)
                self.get_logger().info(
                    f'Step → {math.degrees(self._step):.1f} °')
            if hat[1] == -1 and self._hat_prev[1] != -1: # D-pad DOWN
                self._step = max(self._step - STEP_INC, STEP_MIN)
                self.get_logger().info(
                    f'Step → {math.degrees(self._step):.1f} °')
            self._hat_prev = hat

        # ── Cross → go home ──────────────────────────────────────────────────
        if btn(BTN_CROSS) and not self._moving:
            self.get_logger().info('Cross pressed – moving to home.')
            self._send_goal(list(HOME))
            return

        # ── Don't stack goals ────────────────────────────────────────────────
        if self._moving:
            return

        # ── Compute per-joint deltas ─────────────────────────────────────────
        #   Axes 1 & 3 (stick Y) are inverted: positive push-forward = negative
        delta = [0.0] * 6
        delta[0] =  axis(AXIS_J1) * self._step          # J1
        delta[1] = -axis(AXIS_J2) * self._step          # J2  (inverted)
        delta[2] = -axis(AXIS_J3) * self._step          # J3  (inverted)
        delta[3] =  axis(AXIS_J4) * self._step          # J4

        # L2 / R2 for J5 – triggers go from -1 (idle) → +1 (full press)
        l2 = (axis(AXIS_L2) + 1.0) / 2.0   # normalise to 0–1
        r2 = (axis(AXIS_R2) + 1.0) / 2.0
        if l2 > DEADZONE or r2 > DEADZONE:
            delta[4] = (r2 - l2) * self._step

        # L1 / R1 for J6
        if btn(BTN_L1) or btn(BTN_R1):
            delta[5] = (btn(BTN_R1) - btn(BTN_L1)) * self._step

        # If nothing is being commanded, skip
        if all(abs(d) < 1e-6 for d in delta):
            return

        # Build target
        target = [self._current_joints[i] + delta[i] for i in range(6)]
        self._send_goal(target)

    # ── Send a MoveGroup goal ─────────────────────────────────────────────────
    def _send_goal(self, joints: list[float]):
        goal = MoveGroup.Goal()
        req  = MotionPlanRequest()
        req.group_name                       = 'manipulator'
        req.max_velocity_scaling_factor      = VEL
        req.max_acceleration_scaling_factor  = ACC
        req.allowed_planning_time            = 2.0
        req.num_planning_attempts            = 3

        # Set start state from current joint positions so plans are smooth
        from sensor_msgs.msg import JointState as JS
        from moveit_msgs.msg import RobotState
        rs = RobotState()
        js = JS()
        js.name     = list(JOINT_NAMES)
        js.position = list(self._current_joints)
        rs.joint_state = js
        req.start_state = rs

        constraints = Constraints()
        for name, pos in zip(JOINT_NAMES, joints):
            jc = JointConstraint()
            jc.joint_name     = name
            jc.position       = pos
            jc.tolerance_above = 0.005
            jc.tolerance_below = 0.005
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        req.goal_constraints.append(constraints)
        goal.request              = req
        goal.planning_options     = PlanningOptions()
        goal.planning_options.plan_only = False

        self._moving = True
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by MoveIt.')
            self._moving = False
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self._moving = False


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PS4JogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
