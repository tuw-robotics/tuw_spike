from sensor_msgs.msg import Image, CameraInfo

import rclpy
from rclpy.time import Time
from rclpy.node import Node

import cv_bridge
import cv2
import cv2.aruco

import message_filters

class TrackMarker(Node):
    def __init__(self):
        super().__init__(f"track_marker")
        self.bridge = cv_bridge.CvBridge()

        sub_img = message_filters.Subscriber(self, Image, "camera/image/uncompressed")
        sub_info = message_filters.Subscriber(self, CameraInfo, "camera/camera_info")
        self.sync = message_filters.ApproximateTimeSynchronizer([sub_img, sub_info], 10, 0.2)
        self.sync.registerCallback(self.img_callback)

        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    def img_callback(self, img_msg: Image, info_msg: CameraInfo):
        img = self.bridge.imgmsg_to_cv2(img_msg)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.arucoDict)
        print(corners)
        img = cv2.aruco.drawDetectedMarkers(img, corners, ids)

        cv2.imshow("marker", img)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TrackMarker())
    rclpy.shutdown()

if __name__ == '__main__':
    main()