import glob
import cv2
import rclpy
import os
from cv_bridge import CvBridge
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image

class ImagesNodes(Node):

    def __init__(self):
        super().__init__('image_publisher')

        self.images_path = os.path.join(get_package_share_directory('ias0220_231899'), ('data/images'))

        self.images = glob.glob('*.png', root_dir=self.images_path)

        self.bridge = CvBridge()
        
        self.image_publisher = self.create_publisher(
            Image,
            '/image_raw',
            10)
        self.i = 0
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publisher_callback)
    
    def publisher_callback(self):
        file_path = self.images_path + '/' + self.images[self.i]
        img = cv2.imread(file_path)
        image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        image_msg.header.frame_id = "camera"
        self.i += 1
        if (self.i > 36):
            self.i = 0
        #cv2.imshow('img',img)
        #cv2.waitKey(1000)
        self.image_publisher.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)

    images = ImagesNodes()

    rclpy.spin(images)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    images.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()