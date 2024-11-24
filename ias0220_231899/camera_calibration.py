import numpy as np
import glob
import cv2
import rclpy
import os
from cv_bridge import CvBridge
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class CameraCalibration(Node):

    def __init__(self):
        super().__init__('camera_calibration')

        self.images_path = os.path.join(get_package_share_directory('ias0220_231899'), ('data/images'))

        self.images = glob.glob('*.png', root_dir=self.images_path)

        self.bridge = CvBridge()


        self.imgCount = 0
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.
        self.squareSize = 0.108 # size in meters
        self.objp = np.zeros((6*7,3), np.float32)
        self.objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2) * self.squareSize
        
        self.image_subscriber = self.create_subscription(
            Image,
            '/image_raw',
            self.subscriber_callback,
            10)
        self.image_processed_publisher = self.create_publisher(
            Image,
            '/image_processed',
            10)
        self.camera_info_publisher = self.create_publisher(
            CameraInfo,
            '/camera_info',
            10)
        self.camera_info_msg = CameraInfo()
    
    def subscriber_callback(self, imageMsg):
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        img = self.bridge.imgmsg_to_cv2(imageMsg)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) # convert image from color to gray scale
        # Find the chess board corners using corner detection algorithm
        ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            self.objpoints.append(self.objp)
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            self.imgpoints.append(corners2)
            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
            image_processed_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.image_processed_publisher.publish(image_processed_msg)
            #cv2.imshow('img',img)
            #cv2.waitKey(500)
        #cv2.destroyAllWindows()
        self.imgCount += 1
        if (self.imgCount >= 37):
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, gray.shape[::-1], None,None)
            mean_error = 0
            for i in range(len(self.objpoints)):
                imgpointsKnown, _ = cv2.projectPoints(self.objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv2.norm(self.imgpoints[i], imgpointsKnown, cv2.NORM_L2)/len(imgpointsKnown)
                mean_error += error
            self.get_logger().info(f'Re-projection error: {mean_error}')
            
            
            self.camera_info_msg.header = imageMsg.header
            self.camera_info_msg.height = imageMsg.height
            self.camera_info_msg.width = imageMsg.width
            self.camera_info_msg.distortion_model = "plumb_bob"
            self.camera_info_msg.r = np.eye(3).flatten().tolist()
            self.camera_info_msg.k = mtx.flatten().tolist()
            mtx_zeros = np.zeros((3,1), dtype=mtx.dtype)
            self.camera_info_msg.p = np.append(mtx, mtx_zeros, axis=1).flatten().tolist()
            self.camera_info_msg.d = dist.flatten().tolist()
            self.camera_info_publisher.publish(self.camera_info_msg)
            


def main(args=None):
    rclpy.init(args=args)

    calibrate = CameraCalibration()

    rclpy.spin(calibrate)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    calibrate.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()