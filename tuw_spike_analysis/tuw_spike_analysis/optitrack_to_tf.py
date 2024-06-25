import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry

def quat_mul(a: Quaternion, b: Quaternion):
    return Quaternion(
        w=(a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z),
        x=(a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y),
        y=(a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x),
        z=(a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.x)
    )

class OptitrackToTF(Node):
    def __init__(self):
        super().__init__(f"optitrack_to_tf")

        self.time_offset = self.declare_parameter("time_offset", 0.05).value
        self.sub = self.create_subscription(Odometry, "odom_ground_truth", self.odom_callback, 10)
        self.pub = self.create_publisher(TFMessage, "tf", 10)
        
    def odom_callback(self, msg: Odometry):
        tf = TransformStamped()
        tf.header.frame_id = "map"
        tf_time = Time.from_msg(msg.header.stamp) + Duration(seconds=self.time_offset)
        tf.header.stamp = tf_time.to_msg()
        tf.child_frame_id = "base_odom"
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.rotation = quat_mul(msg.pose.pose.orientation, Quaternion(x=1.0, y=0.0, z=0.0, w=0.0))
        self.pub.publish(TFMessage(transforms=[tf]))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OptitrackToTF())
    rclpy.shutdown()

if __name__ == '__main__':
    main()