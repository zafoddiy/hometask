import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import math

class SquareNode(Node):

    def __init__(self):
        super().__init__('square_node')
        
        self.twist_publisher = self.create_publisher(
            Twist,
            '/diff_cont/cmd_vel',
            10)
        timer_period = 0.5
        self.last_odom_time = self.get_clock().now()
        self.timer = self.create_timer(timer_period, self.publisher_callback)
    
    def publisher_callback(self):
        robot_twist = Twist()
        robot_twist.linear.x = 0.2
        robot_twist.angular.z = 0.0

        now_odom_time = self.get_clock().now()
        dt_tmp = (now_odom_time - self.last_odom_time).to_msg()
        self.dt = float(dt_tmp.sec + dt_tmp.nanosec/1e9)

        if (self.dt >= 4.0):
            self.last_odom_time = self.get_clock().now()
            robot_twist.angular.z = math.pi
            while (self.dt < 2.0):
                now_odom_time = self.get_clock().now()
                dt_tmp = (now_odom_time - self.last_odom_time).to_msg()
                self.dt = float(dt_tmp.sec + dt_tmp.nanosec/1e9)

        self.twist_publisher.publish(robot_twist)

        


def main(args=None):
    rclpy.init(args=args)

    square = SquareNode()

    rclpy.spin(square)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    square.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()