# fanuc_tools

ROS2 Humble package of ready-to-run example tools for the **FANUC CRX-10iA/L** robot.  
Each tool is a single launch command — configuration is handled by YAML files in `config/`.

---

## Package Layout

```
fanuc_tools/
├── config/
│   ├── motion/
│   │   ├── move_joint.yaml       # joint-space A↔B loop
│   │   ├── move_cartesian.yaml   # Cartesian pose A↔B loop
│   │   ├── move_linear.yaml      # straight-line square path
│   │   └── speed_scaling.yaml    # speed override publisher
│   └── tcp/
│       └── read_tcp.yaml         # TCP pose reader
├── fanuc_tools/
│   ├── motion/
│   │   ├── move_joint.py
│   │   ├── move_cartesian.py
│   │   ├── move_linear.py
│   │   ├── speed_scaling.py
│   │   ├── collaborative_speed.py
│   │   └── jog_ps4.py
│   └── tcp/
│       ├── read_tcp.py
│       └── tf_tcp.py
├── launch/
│   ├── motion/
│   │   ├── move_joint.launch.py
│   │   ├── move_cartesian.launch.py
│   │   ├── move_linear.launch.py
│   │   └── speed_scaling.launch.py
│   └── tcp/
│       ├── load_pointer.launch.py
│       └── read_tcp.launch.py
└── urdf/
    └── pointer.urdf.xacro
```

---

## Build

```bash
cd ~/ws_fanuc
colcon build --packages-select fanuc_tools --symlink-install
source install/setup.bash
```

> **Important:** YAML config files are **copied** at build time (not symlinked).  
> After editing any `.yaml` in `config/`, rebuild to apply the changes:
> ```bash
> rm -rf build/fanuc_tools install/fanuc_tools
> colcon build --packages-select fanuc_tools --symlink-install
> source install/setup.bash
> ```

---

## Tools

### 1. `move_joint` — Joint-Space A↔B Loop

Moves the robot back and forth between two joint positions in a continuous loop.  
Config: `config/motion/move_joint.yaml`

| Parameter | Default | Description |
|---|---|---|
| `vel` / `acc` | `0.2` | Velocity / acceleration scaling (0–1) |
| `startup_delay` | `5.0 s` | Wait for MoveIt + controllers before first move |
| `delay_between_moves` | `2.0 s` | Pause at each position |
| `position_a` | all zeros | Home pose — all joints at 0° |
| `position_b` | J1=30°, J2=-25°, J3=40°, J5=15° | Target pose |

All joint values in the YAML are in **degrees**. Conversion to radians is done internally.

**CRX-10iA/L joint directions:**

| Joint | Axis | Positive direction |
|---|---|---|
| J1 | Base rotation | Counter-clockwise from above |
| J2 | Shoulder | Forward / up |
| J3 | Elbow | Up |
| J4 | Wrist roll | Counter-clockwise from front |
| J5 | Wrist pitch | Down |
| J6 | Wrist rotation | Counter-clockwise from front |

```bash
# Mock (simulated)
ros2 launch fanuc_tools move_joint.launch.py use_mock:=true use_rviz:=false

# Real robot
ros2 launch fanuc_tools move_joint.launch.py use_mock:=false use_rviz:=false

# With RViz
ros2 launch fanuc_tools move_joint.launch.py use_mock:=true use_rviz:=true
```

---

### 2. `move_cartesian` — Cartesian Pose A↔B Loop

Moves the TCP (tool centre point) between two Cartesian poses using MoveIt motion planning.  
Uses the `pointer_tcp` frame as the end-effector. Config: `config/motion/move_cartesian.yaml`

| Parameter | Default | Description |
|---|---|---|
| `vel` / `acc` | `0.1` | Velocity / acceleration scaling |
| `delay_between_moves` | `2.0 s` | Pause at each pose |
| `position_tolerance` | `0.001 m` | ~1 mm position tolerance |
| `orientation_tolerance` | `0.01 rad` | ~0.5° orientation tolerance |
| Pose A | x=0.5, y=0.0, z=0.5, roll=180° | Start pose |
| Pose B | x=0.5, y=0.2, z=0.3, roll=180° | Target pose |

Position units: **metres**. Rotation units: **degrees** (roll/pitch/yaw).

```bash
# Mock
ros2 launch fanuc_tools move_cartesian.launch.py use_mock:=true use_rviz:=false

# Real robot
ros2 launch fanuc_tools move_cartesian.launch.py use_mock:=false use_rviz:=false
```

---

### 3. `move_linear` — Cartesian Straight-Line Square

Traces a 200 mm × 200 mm square using `/compute_cartesian_path` (straight-line segments).  
Config: `config/motion/move_linear.yaml`

| Parameter | Default | Description |
|---|---|---|
| `eef_step` | `0.01 m` | Interpolation step along the path |
| `jump_threshold` | `0.0` | Joint-space jump detection (0 = disabled) |
| `delay_between_squares` | `2.0 s` | Pause between laps |

> **Note:** Requires the robot to be pre-positioned near the target workspace.  
> In mock mode starting from all-zeros, `fraction` will be 0.0 and the path will not execute — this is expected behaviour.

```bash
ros2 launch fanuc_tools move_linear.launch.py use_mock:=true use_rviz:=false
```

---

### 4. `speed_scaling` — Speed Override Publisher

Interactive CLI tool. Enter a percentage (0–100) to publish a speed scaling factor to the robot controller. Type `q` to quit.

```bash
ros2 launch fanuc_tools speed_scaling.launch.py

# Or run directly
ros2 run fanuc_tools speed_scaling
```

---

### 5. `collaborative_speed` — Auto Speed Reducer

Subscribes to GPIO/force data and automatically reduces speed when a human is detected nearby. Starts and waits for hardware input — no interaction needed.

```bash
ros2 run fanuc_tools collaborative_speed
```

---

### 6. `load_pointer` — Pointer Tool TF Publisher

Publishes the `pointer_link` and `pointer_tcp` TF frames for the pointer tool attachment.  
Required before running `move_cartesian` or `move_linear`.

```bash
ros2 launch fanuc_tools load_pointer.launch.py
```

---

### 7. `read_tcp` — TCP Pose Reader

Reads and prints the current TCP (tool centre point) pose from TF.

```bash
ros2 launch fanuc_tools read_tcp.launch.py
```

---

## Quick Reference

| Tool | Launch command | Config file |
|---|---|---|
| Joint loop | `ros2 launch fanuc_tools move_joint.launch.py` | `config/motion/move_joint.yaml` |
| Cartesian loop | `ros2 launch fanuc_tools move_cartesian.launch.py` | `config/motion/move_cartesian.yaml` |
| Linear square | `ros2 launch fanuc_tools move_linear.launch.py` | `config/motion/move_linear.yaml` |
| Speed scaling | `ros2 launch fanuc_tools speed_scaling.launch.py` | `config/motion/speed_scaling.yaml` |
| Load pointer | `ros2 launch fanuc_tools load_pointer.launch.py` | — |
| Read TCP | `ros2 launch fanuc_tools read_tcp.launch.py` | `config/tcp/read_tcp.yaml` |

All motion launch files accept:

| Argument | Values | Default |
|---|---|---|
| `use_mock` | `true` / `false` | `true` |
| `use_rviz` | `true` / `false` | `false` |
