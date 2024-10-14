# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from encoders_interfaces.msg import Counter

import math
import transforms3d


class OdometerNode(Node):

    def __init__(self):
        super().__init__('odometry')
        self.max_cntr = 508.8

        self.odomSubscription = self.create_subscription(
            Counter,
            '/encoders_ticks',
            self.listener_odomCallback,
            10)

        self.odomPublisher = self.create_publisher(
            Odometry,
            '/my_odom',
            10)
        self.odom_msg = Odometry()

        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"

        self.odom_msg.twist.twist.linear.x = 0.0
        self.odom_msg.twist.twist.angular.z = 0.0

        self.odom_msg.pose.pose.position.x = 0.0
        self.odom_msg.pose.pose.position.y = 0.0

        self.odom_msg.pose.pose.orientation.w = 0.0
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0

        self.prev_y = 0.0
        self.prev_x = 0.0
        self.robot_prev_angle = 0.0
        self.prev_left = 0.0
        self.prev_right = 0.0
        self.prev_time = 0.0

        self.i = 0

        self.odomSubscription  # prevent unused variable warning

    def calc_wheel_angle(self, curr_wheel, prev_wheel):
        if (self.i == 0):
            cntr_diff = 0 - curr_wheel
        else:
            cntr_diff = prev_wheel - curr_wheel
        if (abs(cntr_diff) > (self.max_cntr / 2) and cntr_diff < 0):
            cntr_diff = prev_wheel - curr_wheel + self.max_cntr
        elif (abs(cntr_diff) > (self.max_cntr / 2) and cntr_diff > 0):
            cntr_diff = prev_wheel - curr_wheel - self.max_cntr
        wheel_angle = (cntr_diff / self.max_cntr) * 2 * math.pi
        return (wheel_angle)


    def listener_odomCallback(self, counterMsg):
        # Initialization of wheel counters and current time difference
        curr_left = counterMsg.count_left
        curr_right = counterMsg.count_right
        curr_time = self.get_clock().now().nanoseconds/1e9
        if (self.i == 0):
            time_diff = curr_time
        else:
            time_diff = curr_time - self.prev_time

        # Calculating wheel angles and angular velocities
        left_wheel_angle = 0 - self.calc_wheel_angle(curr_left, self.prev_left)
        right_wheel_angle = self.calc_wheel_angle(curr_right, self.prev_right)
        left_wheel_angular_vel = left_wheel_angle / time_diff
        right_wheel_angular_vel = right_wheel_angle / time_diff

        # Calculating wheel linear velocities
        wheel_radius = 0.036
        left_lin_vel = wheel_radius * left_wheel_angular_vel
        right_lin_vel = wheel_radius * right_wheel_angular_vel
        
        # Calculating robot velocities and their integrals
        ref_point_dist = 0.175
        robot_linear_vel = (left_lin_vel + right_lin_vel) / 2
        robot_curve = robot_linear_vel * time_diff
        robot_angular_vel = (right_lin_vel - left_lin_vel) / (2 * ref_point_dist)
        robot_angle_delta = robot_angular_vel * time_diff
        if (self.i == 0):
            robot_curr_angle = robot_angle_delta
        else:
            robot_curr_angle = self.robot_prev_angle + robot_angle_delta

        # Robot current x and y coordinates
        if (self.i == 0):
            trig_val = robot_angle_delta / 2
            curr_x = robot_curve * math.cos(trig_val)
            curr_y = robot_curve * math.sin(trig_val)
        else:
            trig_val = self.robot_prev_angle + robot_angle_delta / 2
            curr_x = self.prev_x + robot_curve * math.cos(trig_val)
            curr_y = self.prev_y + robot_curve * math.sin(trig_val)


        self.odom_msg.header.stamp = self.get_clock().now().to_msg()

        self.odom_msg.twist.twist.linear.x = robot_linear_vel
        self.odom_msg.twist.twist.angular.z = robot_angular_vel

        self.odom_msg.pose.pose.position.x = curr_x
        self.odom_msg.pose.pose.position.y = curr_y

        quaternions = transforms3d.euler.euler2quat(0, 0, robot_curr_angle, axes='sxyz')

        self.odom_msg.pose.pose.orientation.w = quaternions[0]
        self.odom_msg.pose.pose.orientation.x = quaternions[1]
        self.odom_msg.pose.pose.orientation.y = quaternions[2]
        self.odom_msg.pose.pose.orientation.z = quaternions[3]

        self.odomPublisher.publish(self.odom_msg)


        self.prev_y = curr_y
        self.prev_x = curr_x
        self.robot_prev_angle = robot_curr_angle
        self.prev_left = curr_left
        self.prev_right = curr_right
        self.prev_time = curr_time
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    odometer = OdometerNode()

    rclpy.spin(odometer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odometer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
