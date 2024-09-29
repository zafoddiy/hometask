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
import random
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Vector3


class Walker(Node):

    def __init__(self):
        super().__init__('walker')
        self.velocityPublisher_ = self.create_publisher(Vector3, 'velocity', 2)
        self.nametimePublisher_ = self.create_publisher(String, '/name_and_time', 2)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.velMsg = Vector3()

    def timer_callback(self):
        velList = [-1.0, 0.0, 1.0]
        x_coord = random.choice(velList)
        y_coord = random.choice(velList)
        self.velMsg.x = x_coord
        self.velMsg.y = y_coord
        self.velocityPublisher_.publish(self.velMsg)
        nametimeMsg = String()
        nametimeMsg.data = '231899' + ',' + str(self.get_clock().now().nanoseconds/1e9)
        self.nametimePublisher_.publish(nametimeMsg)
        self.get_logger().info('ID and current time: "%s"' % nametimeMsg.data)
        self.get_logger().info('Velocity for 0.5 seconds:\nx: "%0.2f"\ny: "%0.2f"\nz: "%0.2f"' % (self.velMsg.x, self.velMsg.y, self.velMsg.z))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    walker = Walker()

    rclpy.spin(walker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    walker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
