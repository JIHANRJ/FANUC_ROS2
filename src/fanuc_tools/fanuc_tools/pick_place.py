import rclpy
import time
from fanuc_tools.crx_interface import CRX


def main():

    rclpy.init()
    robot = CRX()

    rclpy.spin_once(robot, timeout_sec=1.0)

    home = [0.6, 0.0, 0.9, 0.0, 1.57, 0.0]

    pre_pick = [0.7, -0.2, 0.95, 0.0, 1.57, 0.0]
    pick     = [0.7, -0.2, 0.85, 0.0, 1.57, 0.0]

    pre_place = [0.7, 0.2, 0.95, 0.0, 1.57, 0.0]
    place     = [0.7, 0.2, 0.85, 0.0, 1.57, 0.0]

    # Show markers
    robot.show_point(home, "home", (0.0,1.0,0.0))
    robot.show_point(pre_pick, "pre_pick", (1.0,1.0,0.0))
    robot.show_point(pick, "pick", (1.0,0.0,0.0))
    robot.show_point(pre_place, "pre_place", (0.0,1.0,1.0))
    robot.show_point(place, "place", (0.0,0.0,1.0))

    rclpy.spin_once(robot, timeout_sec=0.5)
    robot.add_box("pedestal", [0.3, 0.3, 0.5], [0.7, 0.0, 0.25])

    time.sleep(1.0)

    robot.move_pose(home)
    robot.move_pose(pre_pick)
    robot.move_pose(pick)

    print("Closing gripper")
    time.sleep(1)

    robot.move_pose(pre_pick)
    robot.move_pose(pre_place)
    robot.move_pose(place)

    print("Opening gripper")
    time.sleep(1)

    robot.move_pose(pre_place)
    robot.move_pose(home)

    print("Pick and place complete")

    rclpy.shutdown()


if __name__ == "__main__":
    main()