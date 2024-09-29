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

from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose


class PositionCalculator(Node):

    def __init__(self):
        super().__init__('position_calculator')
        self.coord = Pose()
        self.coord.position.z = 0.0
        self.nameandtimeSubscription = self.create_subscription(
            String,
            '/name_and_time',
            self.listener_nametimeCallback,
            2)
        self.velSubscription = self.create_subscription(
            Vector3,
            'velocity',
            self.listener_velCallback,
            2)
        self.velSubscription  # prevent unused variable warning

    def listener_nametimeCallback(self, nametimeMsg):
        self.get_logger().info('Student "%s" contacted me, and told me that the current time is: "%s"'
                                % (nametimeMsg.data[:6], nametimeMsg.data[7:]))
    def listener_velCallback(self, velMsg):
        self.coord.position.x += velMsg.x * 0.5
        self.coord.position.y += velMsg.y * 0.5
        self.get_logger().info('The position of the walker is:\nx:"%0.2f"\ny:"%0.2f"\nz:"%0.2f"'
                                % (self.coord.position.x, self.coord.position.y, self.coord.position.z))


def main(args=None):
    rclpy.init(args=args)

    position_calculator = PositionCalculator()

    rclpy.spin(position_calculator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    position_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
