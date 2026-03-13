import rclpy
from fanuc_tools.legacy.crx_interface import CRX

def main():
    rclpy.init()

    robot = CRX()
    rclpy.spin_once(robot, timeout_sec=1.0)

    robot.move_joint([0.3,0.4,-1.2,1.2,-0.3,0.5])
    robot.move_pose([0.7,-0.15,0.955,1.0,0.0,0.0])

    rclpy.shutdown()

if __name__ == "__main__":
    main()