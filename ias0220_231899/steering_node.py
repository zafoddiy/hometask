import rclpy
import transforms3d.euler
from rclpy.node import Node

from sensor_msgs.msg import Imu, Range
from geometry_msgs.msg import Twist, Vector3

class SteeringNode(Node):

    def __init__(self):
        super().__init__('steering_node')
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        self.imu_rpy = [0, 0, 0]
        self.lin_vel = Vector3()
        self.ang_vel = Vector3()
        self.dist_multiplier = 0
        self.dist_subscriber = self.create_subscription(
            Range,
            '/distance',
            self.dist_callback,
            10)
        self.dist_check = False
        self.twist_publisher = self.create_publisher(
            Twist,
            '/tb4_34/cmd_vel',
            10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publisher_callback)
    
    def imu_callback(self, imu_msg):
        imu_quat = imu_msg.orientation
        self.imu_rpy = transforms3d.euler.quat2euler([imu_quat.w, imu_quat.x, imu_quat.y, imu_quat.z], axes='sxyz')
        self.lin_vel.x = self.imu_rpy[1] * 5
        self.ang_vel.z = self.imu_rpy[0] * 5
    
    def dist_callback(self, dist_msg):
        self.dist_multiplier = dist_msg.range
        if dist_msg.range > 0.2:
            self.dist_check = True
        else:
            self.dist_check = False
    
    def publisher_callback(self):
        robot_twist = Twist()
        if self.dist_check == False:
            robot_twist.angular.z = self.ang_vel.z * float(self.dist_multiplier)
            robot_twist.linear.x = self.lin_vel.x * float(self.dist_multiplier)
        else:
            robot_twist.angular.z = 0.0
            robot_twist.linear.x = 0.0
        self.twist_publisher.publish(robot_twist)

        


def main(args=None):
    rclpy.init(args=args)

    steering = SteeringNode()

    rclpy.spin(steering)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    steering.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()