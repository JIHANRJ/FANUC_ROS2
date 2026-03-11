"""
speed_scaling.py
================
Quick run:
    source /opt/ros/humble/setup.bash && source ~/ws_fanuc/install/setup.bash
    ros2 launch fanuc_tools move_joint.launch.py use_mock:=true use_rviz:=false
    ros2 run fanuc_tools speed_scaling

Example: Dynamically control the FANUC CRX-10iA/L execution speed
         while a trajectory is running.

What it does:
    Continuously reads a speed value (0-100) from terminal input and
    publishes it to /speed_scaling_factor. The robot's
    scaled_joint_trajectory_controller adjusts execution speed in real time.

When to use it:
    - Slow down the robot near sensitive areas
    - Pause motion completely (set to 0)
    - Resume motion (set back to a positive value)
    - Human-robot collaboration scenarios

Key concepts:
    - /speed_scaling_factor [std_msgs/Int32]: the topic the SJTC listens to
    - Value range: 0.0 (stop) to 100.0 (full speed)
    - This affects execution speed, not the planned path

Run alongside move_joint:
    Terminal 1 (start MoveIt):
        ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \
            robot_model:=crx10ia_l use_mock:=true

    Terminal 2 (start a long trajectory):
        ros2 run fanuc_tools move_joint --ros-args \
            -p vel:=0.5 -p acc:=0.5 \
            -p target_joints.joint_1:=1.5708 \
            -p target_joints.joint_2:=-0.7854 \
            -p target_joints.joint_3:=1.5708 \
            -p target_joints.joint_4:=0.0 \
            -p target_joints.joint_5:=0.7854 \
            -p target_joints.joint_6:=0.0

    Terminal 3 (control speed while robot is moving):
        ros2 run fanuc_tools speed_scaling

Usage:
    Type a number 0-100 and press Enter to change speed.
    Type 'q' to quit.
"""

import rclpy
from rclpy.node import Node
import threading
from std_msgs.msg import Int32

class SpeedScalingNode(Node):
    """
    Publishes speed scaling values to /speed_scaling_factor.
    Reads input from the terminal in a separate thread so the
    ROS 2 event loop keeps running in the main thread.
    """

    def __init__(self):
        super().__init__('speed_scaling_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('initial_speed', 100.0)  # start at full speed
        self.declare_parameter('publish_rate', 10.0)    # Hz — how often to publish

        self.speed = float(self.get_parameter('initial_speed').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        # ── Publisher ────────────────────────────────────────────────────────
        # The scaled_joint_trajectory_controller subscribes to this topic.
        # We publish at a fixed rate so the controller always has a fresh value.
        self.publisher = self.create_publisher(
            Int32,
            '/speed_scaling_factor',
            10
        )

        # ── Timer ────────────────────────────────────────────────────────────
        # Publishes the current speed value at the configured rate.
        # Separate from the input thread so publishing continues smoothly
        # even while waiting for user input.
        self.create_timer(1.0 / publish_rate, self.publish_speed)

        self.get_logger().info('Speed Scaling Node started.')
        self.get_logger().info(f'Publishing to /speed_scaling_factor at {publish_rate} Hz')
        self.get_logger().info(f'Initial speed: {self.speed}%')
        self.get_logger().info('─' * 40)
        self.get_logger().info('Type a value (0-100) and press Enter to change speed.')
        self.get_logger().info('  0   = pause robot completely')
        self.get_logger().info('  50  = half speed')
        self.get_logger().info('  100 = full speed')
        self.get_logger().info('  q   = quit')
        self.get_logger().info('─' * 40)

        # ── Input thread ─────────────────────────────────────────────────────
        # Terminal input is blocking — if we called input() in the main thread
        # it would block rclpy.spin() and ROS 2 would stop processing callbacks.
        # Running input in a separate thread keeps both working simultaneously.
        self.input_thread = threading.Thread(
            target=self.read_input,
            daemon=True   # dies automatically when main program exits
        )
        self.input_thread.start()

    def publish_speed(self):
        """
        Called by timer at the configured rate.
        Publishes the current speed value to /speed_scaling_factor.
        """
        msg = Int32()
        msg.data = int(round(self.speed))
        self.publisher.publish(msg)

    def read_input(self):
        """
        Runs in a separate thread.
        Continuously reads user input and updates self.speed.
        The timer callback picks up the new value on its next tick.
        """
        while rclpy.ok():
            try:
                user_input = input('Speed (0-100): ').strip()

                # Quit
                if user_input.lower() == 'q':
                    self.get_logger().info('Quitting...')
                    rclpy.shutdown()
                    break

                # Parse and validate
                value = float(user_input)

                if value < 0.0 or value > 100.0:
                    self.get_logger().warn(
                        f'Value {value} out of range. Enter a value between 0 and 100.'
                    )
                    continue

                # Update speed — timer will publish this on next tick
                self.speed = value

                # Human readable feedback
                if value == 0.0:
                    self.get_logger().info('Robot PAUSED.')
                elif value == 100.0:
                    self.get_logger().info('Robot at FULL speed.')
                else:
                    self.get_logger().info(f'Speed set to {value}%')

            except ValueError:
                self.get_logger().warn(
                    f'Invalid input. Enter a number between 0-100 or q to quit.'
                )
            except EOFError:
                # Happens if stdin is closed
                break


def main(args=None):
    rclpy.init(args=args)
    node = SpeedScalingNode()

    try:
        # spin() runs the ROS 2 event loop in the main thread
        # input() runs in the background thread simultaneously
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
