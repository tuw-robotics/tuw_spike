import rclpy.qos

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from tf2_ros import Buffer, TransformListener, TransformException

import cv_bridge
import cv2
import numpy as np

from scipy.spatial.transform import Rotation

import tf2_geometry_msgs # Needed for transform registration
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header

class ExtrinsicCalibrationNode(Node):
    def __init__(self):
        super().__init__(f"extrinsic_calibration")
        self.bridge = cv_bridge.CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sub_img = message_filters.Subscriber(self, Image, "image", qos_profile=rclpy.qos.qos_profile_sensor_data)
        sub_info = message_filters.Subscriber(self, CameraInfo, "camera_info", qos_profile=rclpy.qos.qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([sub_img, sub_info], 10, 0.2)
        self.sync.registerCallback(self.img_callback)

        self.square_size = 0.036
        self.checkerboard_origin_world = np.array(((0.255, 0.080, 0.005)), dtype=float)

        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.objp = np.zeros((6*7,3), np.float32)
        self.objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2) * self.square_size

    def img_callback(self, img_msg: Image, info_msg: CameraInfo):
        img = self.bridge.imgmsg_to_cv2(img_msg)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (7,6), None)
        dist = np.array(info_msg.d)
        k = np.array(info_msg.k).reshape((3,3))
        cv2.drawChessboardCorners(img, (7,6), corners, found)
        if found:
            refined_corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), self.criteria)
            # Estimate pose
            ret, rvec, tvec = cv2.solvePnP(self.objp, refined_corners, k, dist)
            # Convert to transformation matrix
            rot_mat, _ = cv2.Rodrigues(rvec)
            transform = np.zeros((4,4), dtype=float)
            transform[0:3,0:3] = rot_mat
            transform[0:3,3:4] = tvec
            transform[3,3] = 1
            # Flip Z axis up
            transform = transform @ [
                [ 1.0,  0.0,  0.0, 0.0],
                [ 0.0, -1.0,  0.0, 0.0],
                [ 0.0,  0.0, -1.0, 0.0],
                [ 0.0,  0.0,  0.0, 1.0],
            ]
            # Display flipped system
            rvec, _ = cv2.Rodrigues(transform[0:3, 0:3])
            tvec = transform[0:3,3:4]
            img = cv2.drawFrameAxes(img, k, dist, rvec, tvec, self.square_size)
            # Invert transform to get checker_board_origin -> camera transform
            cam_transform = np.linalg.inv(transform)
            # Add checkerboard origin in base_odom system to get base_odom -> camera transform
            cam_transform[0:3,3] += self.checkerboard_origin_world
            self.get_logger().info(f"Transform:\n{cam_transform}")


            t = Pose()
            t.orientation.x, t.orientation.y, t.orientation.z, t.orientation.w = tuple(
                Rotation.from_matrix(cam_transform[0:3,0:3]).as_quat()
            )
            t.position.x, t.position.y, t.position.z = tuple(
                cam_transform[0:3,3]
            )

            try:
                t_camera_to_optical: PoseStamped = self.tf_buffer.transform(
                    PoseStamped(
                        pose=t,
                        header=Header(
                            stamp=Time(nanoseconds=0).to_msg(),
                            frame_id="base_odom"
                        )
                    ),
                    "camera_optical"
                )
                angles = Rotation.from_quat((
                    t_camera_to_optical.pose.orientation.x,
                    t_camera_to_optical.pose.orientation.y,
                    t_camera_to_optical.pose.orientation.z,
                    t_camera_to_optical.pose.orientation.w
                )).as_euler("xyz")
                offset = t_camera_to_optical.pose.position
                self.get_logger().info("Transform relative to camera link:\n"
                                    f"translation = ({offset.x:.4f} {offset.y:.4f} {offset.z:.4f})\n"
                                    f"rotation (rpy) = ({angles[0]:.3f} {angles[1]:.3f} {angles[2]:.3f})")
            except TransformException as ex:
                self.get_logger().warn(f"Could not transform from base_odom to camera frame: {ex}")

        img = cv2.undistort(img, k, dist)
        cv2.imshow("marker", img)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ExtrinsicCalibrationNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()