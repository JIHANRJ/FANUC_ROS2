# Sandbox

Exploratory scripts for learning and testing the FANUC ROS 2 driver.
These are not production code — they are self-contained examples
that can be run directly with `python3` without building.

## Prerequisites

Terminal 1 must always be running before any sandbox script:
```bash
ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \
    robot_model:=crx10ia_l use_mock:=true
```

---

## Scripts

### `read_joint_states.py`
Subscribes to `/joint_states` and displays current joint positions
in a neat in-place table.

**Run:**
```bash
python3 sandbox/read_joint_states.py
```

**Output:**
```
┌────────┬──────────────┬──────────────┐
│ Joint  │     Radians  │     Degrees  │
├────────┼──────────────┼──────────────┤
│ J1     │  -0.6876 rad │   -39.39 deg │
│ J2     │  +1.0146 rad │   +58.13 deg │
│ J3     │  +2.5762 rad │  +147.60 deg │
│ J4     │  +2.4539 rad │  +140.60 deg │
│ J5     │  +1.5637 rad │   +89.60 deg │
│ J6     │  +3.1474 rad │  +180.33 deg │
└────────┴──────────────┴──────────────┘
```

#### `JointStateReader` — Function Catalogue

| Function | Input | Sample | Returns |
|---|---|---|---|
| `get_joints()` | none | `reader.get_joints()` | `{'J1': -0.68, 'J2': 1.01, 'J3': 2.57, 'J4': 2.45, 'J5': 1.56, 'J6': 3.14}` or `None` if not ready |
| `get_joint()` | `name: str` | `reader.get_joint('J1')` | `float` — position in radians, or `None` |
| `is_ready()` | none | `reader.is_ready()` | `True` once first `/joint_states` message received, `False` before |
| `print_joints()` | none | `reader.print_joints()` | `None` — prints in-place table to terminal |

**Use from another script:**
```python
from read_joint_states import JointStateReader

reader = JointStateReader()

# Check ready before reading
if reader.is_ready():
    joints = reader.get_joints()    # all joints as dict
    j1     = reader.get_joint('J1') # single joint in radians
```

---

### `command_joints.py`
Sends a joint goal to MoveIt and prints final result.

**Run:**
```bash
python3 sandbox/command_joints.py
```

**Output:**
```
[joint_commander]: JointCommander ready.
[joint_commander]: Goal ACCEPTED — moving...
[joint_commander]: ✓ MOTION COMPLETE
[joint_commander]:   J1: +0.5236 rad (+30.00 deg)
[joint_commander]:   J2: -0.7854 rad (-45.00 deg)
[joint_commander]:   J3: +1.5708 rad (+90.00 deg)
[joint_commander]:   J4: +0.0000 rad (+0.00 deg)
[joint_commander]:   J5: +0.7854 rad (+45.00 deg)
[joint_commander]:   J6: +0.0000 rad (+0.00 deg)
```

#### `JointCommander` — Function Catalogue

| Function | Input | Sample | Returns |
|---|---|---|---|
| `send_goal()` | `target_joints: list[float]`, `vel: float=0.1`, `acc: float=0.1` | `commander.send_goal([0.5, -0.7, 1.5, 0.0, 0.7, 0.0], vel=0.2)` | `None` — result delivered via `on_result` callback |
| `get_current_joints()` | none | `commander.get_current_joints()` | `{'J1': 0.12, 'J2': -0.45, 'J3': 1.23, 'J4': 0.0, 'J5': 0.78, 'J6': 0.0}` or `None` |
| `get_error_to_target()` | none | `commander.get_error_to_target()` | `{'J1': 0.03, 'J2': 0.12, 'J3': 0.01, 'J4': 0.0, 'J5': 0.05, 'J6': 0.0}` or `None` |
| `is_ready()` | none | `commander.is_ready()` | `True` once `/joint_states` received, `False` before |
| `on_result` | `fn(error_code: int, final_joints: list[float])` | `commander.on_result = my_fn` | Assign a function — called automatically when motion completes |

**Error codes:**

| Code | Meaning |
|---|---|
| `1` | SUCCESS — goal reached |
| `-1` | FAILURE — generic error |
| `99999` | GOAL REJECTED — unreachable or in collision |

**Use from another script:**
```python
from command_joints import JointCommander

commander = JointCommander()

# Simple — use default result handler
commander.send_goal([0.5, -0.7, 1.5, 0.0, 0.7, 0.0], vel=0.2)

# Custom result handler
def on_done(error_code, final_joints):
    if error_code == 1:
        print(f'Done! J1 is now {final_joints[0]:.4f} rad')

commander.on_result = on_done
commander.send_goal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=0.1)
```

---

## Notes

- All joint values are in **radians** unless stated otherwise
- Velocity and acceleration scaling: `0.0` = stopped, `1.0` = full speed
- Always test with `use_mock:=true` before connecting real hardware
- Joint names for CRX-10iA/L: `J1, J2, J3, J4, J5, J6`

---

### `joint_sequence_demo.py`

Interactive recorder/replayer for joint waypoints.

**What it does:**
- Records the current robot joint state as named points at record time
- Saves and loads those points from JSON
- Replays the points through MoveIt so you can watch the motion in RViz

**Run in mock mode:**
```bash
source /opt/ros/humble/setup.bash
source ~/ws_fanuc/install/setup.bash
env -u GTK_PATH -u GTK_PATH_VSCODE_SNAP_ORIG \
    ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \
    robot_model:=crx10ia use_mock:=true
```

Open a second terminal in `src/fanuc_tools/fanuc_tools` and run:
```bash
python3 sandbox/joint_sequence_demo.py --file sandbox/joint_sequence.json
```

Replay is tuned for smoother motion by default:
- velocity scaling defaults to `0.05`
- acceleration scaling defaults to `0.05`
- joint-space interpolation defaults to `5 deg` max step size

For even smoother transitions, lower the interpolation step:
```bash
python3 sandbox/joint_sequence_demo.py --file sandbox/joint_sequence.json --interp-step-deg 2.0
```

To see a ready-made RViz demo sequence replay automatically:
```bash
python3 sandbox/joint_sequence_demo.py --file sandbox/joint_sequence.json --demo
```

Menu:
- `r` record current joint state
- `l` list recorded points
- `s` save points to JSON
- `o` load points from JSON
- `p` replay the recorded sequence
- `c` clear points
- `q` quit

---

### `plot_joint_velocities_live.py`

Live plot of joint velocities from `/joint_states` to help diagnose jerky motion.

**What it does:**
- Displays six live plots (J1..J6) in rad/s
- Uses `JointState.velocity` when available
- Falls back to velocity estimation from position deltas if needed

**Run:**
```bash
python3 sandbox/plot_joint_velocities_live.py --history-sec 10
```

If your joint names differ, pass them explicitly:
```bash
python3 sandbox/plot_joint_velocities_live.py --joints J1,J2,J3,J4,J5,J6
```