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
            '/contoured_image',
            10)
        self.i = 0

    def subscriber_callback(self, imageMsg):
        img = self.bridge.imgmsg_to_cv2(imageMsg, desired_encoding='bgr8')
        img_HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower_red = np.array([-10, 100, 100])
        upper_red = np.array([5, 255, 255])
        img_masked = cv.inRange(img_HSV, lower_red, upper_red)
        contours, hierarchy = cv.findContours(
            img_masked, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        contours_poly = [None]*len(contours)
        centers = [None]*len(contours)
        radius = [None]*len(contours)
        for i, c in enumerate(contours):
            contours_poly[i] = cv.approxPolyDP(c, 3, True)
            centers[i], radius[i] = cv.minEnclosingCircle(contours_poly[i])

        try:
            cv.circle(img, (int(centers[1][0]), int(
                centers[1][1])), int(radius[1]), (0, 255, 0), 2)
        except Exception as e:
            self.get_logger().info("No object detected")
        image_debug_msg = self.bridge.cv2_to_imgmsg(
            img, encoding='bgr8')
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
