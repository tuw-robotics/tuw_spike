import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, Twist

TRAJECTORY_UPDATE_PERIOD = 0.01

class TestTrajectoryDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_driver")

        # change to 0.1 for smaller circles
        self.velocity = self.declare_parameter("velocity", 0.3).get_parameter_value().double_value
        self.center_dist = self.declare_parameter("center_dist", 0.3).get_parameter_value().double_value
        self.cross_alpha = self.declare_parameter("cross_alpha", math.radians(5.0)).get_parameter_value().double_value
        self.startup_delay = self.declare_parameter("startup_delay", 1.0).get_parameter_value().double_value

        self.radius = self.center_dist / 2.0 * math.cos(self.cross_alpha)
        self.straight = self.center_dist * math.sin(self.cross_alpha)
        self.angular_velocity = self.velocity / self.radius

        self.time_straight = self.straight / self.velocity
        self.time_curve = 2.0 * (math.pi - self.cross_alpha) / self.angular_velocity
        self.total_time = 2.0 * self.time_straight + 2.0 * self.time_curve

        self.time = -self.startup_delay

        # # counter is used to keep track of the number of double circles that are done
        # self.counter = 1

        self.pub_vel = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.trajectory_timer = self.create_timer(TRAJECTORY_UPDATE_PERIOD, self.update_trajectory)
        self.update_trajectory()

    def update_trajectory(self):
        twist = Twist()

        if self.time >= 0.0:
            twist.linear.x = self.velocity

        if 0.5*self.time_straight + 0.0*self.time_curve <= self.time \
            and self.time < 0.5*self.time_straight + 1.0*self.time_curve:
            twist.angular.z = self.angular_velocity
        elif 1.5*self.time_straight + 1.0*self.time_curve <= self.time \
            and self.time < 1.5*self.time_straight + 2.0*self.time_curve:
            twist.angular.z = -self.angular_velocity

        self.pub_vel.publish(TwistStamped(
            twist=twist,
            header=Header(stamp=self.get_clock().now().to_msg())
        ))

        self.time += TRAJECTORY_UPDATE_PERIOD
        if self.time > self.total_time:
            # # uncomment to stop after one double circle drive
            # self.get_logger().info(f'Finished circle: {self.counter}')
            # self.counter += 1
            # self.destroy_timer(self.trajectory_timer)
            self.time -= self.total_time

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TestTrajectoryDriverNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
