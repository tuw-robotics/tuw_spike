import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

from .pose_logger import PoseLogger

def main(args=None):
    with open("trajectory_est.csv", "w") as f:
        rclpy.init(args=args)
        rclpy.spin(PoseLogger(f, "amcl_pose", PoseWithCovarianceStamped))
        rclpy.shutdown()

if __name__ == '__main__':
    main()
