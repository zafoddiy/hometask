import cv2 as cv
import rclpy
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image

class ObjectsNodes(Node):

    def __init__(self):
        super().__init__('object_recognition')

        self.bridge = CvBridge()
        
        self.object_subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.subscriber_callback,
            10)
        self.debug_publisher = self.create_publisher(
            Image,
            '/debug_image',
            10)
        self.i = 0
    
    def subscriber_callback(self, imageMsg):
        img = self.bridge.imgmsg_to_cv2(imageMsg)
        self.get_logger().info("Starting process")
        img_HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower_blue = np.array([-10,100,100])
        upper_blue = np.array([10,255,255])
        img_masked = cv.inRange(img_HSV, lower_blue, upper_blue)
        result = cv.bitwise_and(img,img, mask= img_masked)
        image_debug_msg = self.bridge.cv2_to_imgmsg(result)
        self.get_logger().info("Publishing message")
        self.debug_publisher.publish(image_debug_msg)


def main(args=None):
    rclpy.init(args=args)

    objects = ObjectsNodes()

    rclpy.spin(objects)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    objects.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()