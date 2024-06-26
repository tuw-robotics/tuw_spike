import rclpy
from nav_msgs.msg import Odometry

from .pose_logger import PoseLogger

def main(args=None):
    with open("trajectory_odom.csv", "w") as f:
        rclpy.init(args=args)
        rclpy.spin(PoseLogger(f, "odom", Odometry))
        rclpy.shutdown()

if __name__ == '__main__':
    main()
