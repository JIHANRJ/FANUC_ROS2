"""
===============================================================================
FANUC ROS2 State Monitor
===============================================================================

DESCRIPTION:
------------
This node prints either:

1) Joint angles (J1–J6) as a dictionary
   OR
2) End-effector (flange) pose as a dictionary

Joint Output Format:
--------------------
{
    "J1": value,
    "J2": value,
    ...
    "J6": value
}

EOAT / Flange Output Format:
----------------------------
{
    "x": value,
    "y": value,
    "z": value,
    "w": quaternion_w,
    "p": pitch (radians),
    "r": roll (radians)
}

===============================================================================
HOW TO BUILD
===============================================================================

cd ~/ws_fanuc
colcon build
source install/setup.bash

(Always source workspace in every new terminal)

===============================================================================
HOW TO RUN
===============================================================================

First launch MoveIt:

ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \
robot_model:=crx10ia_l use_mock:=true

Then in another terminal:

# Joint Mode
ros2 run fanuc_tools monitor_state --ros-args -p mode:=joint

# Flange / EOAT Mode
ros2 run fanuc_tools monitor_state --ros-args -p mode:=eoat

===============================================================================
NOTES
===============================================================================

• Joint values are in radians.
• Roll/Pitch are in radians.
• TF frame used: base_link → flange
• Default mode is 'joint' if not specified.
• Must have /move_group and robot_state_publisher running.

===============================================================================
"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration
import math

def quaternion_to_rpy(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class RobotMonitor(Node):

    def __init__(self):
        super().__init__('robot_monitor')

        self.declare_parameter('mode', 'joint')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        print(f"Running in mode: {self.mode}")

        if self.mode == "joint":
            self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_callback,
                10)

        elif self.mode == "eoat":
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_timer(1.0, self.print_flange_pose)

        else:
            print("Invalid mode. Use 'joint' or 'eoat'.")

    # ------------------------------
    # Joint Mode (J1, J2, ... J6)
    # ------------------------------
    def joint_callback(self, msg):
        joint_dict = {}

        for i, position in enumerate(msg.position):
            joint_name = f"J{i+1}"
            joint_dict[joint_name] = position

        print(joint_dict)

    # ------------------------------
    # EOAT Mode (x y z w p r)
    # ------------------------------
    def print_flange_pose(self):
        try:
            if not self.tf_buffer.can_transform(
                    'base_link',
                    'flange',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)):
                return

            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'flange',
                rclpy.time.Time()
            )

            t = transform.transform.translation
            q = transform.transform.rotation

            roll, pitch, yaw = quaternion_to_rpy(
                q.x, q.y, q.z, q.w
            )

            pose_dict = {
                "x": t.x,
                "y": t.y,
                "z": t.z,
                "w": yaw,     # yaw (Z)
                "p": pitch,   # pitch (Y)
                "r": roll     # roll (X)
            }

            print(pose_dict)

        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()