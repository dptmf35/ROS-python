#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()
fx = 1431.253265
fy = 1433.305237
cx = 593.392302
cy = 329.256042
k1 = 0.027655
k2 = 0.901755
k3 = 0
p1 = -0.003198
p2 = -0.013385
dist = np.array([k1, k2, p1, p2, k3])
mtx = np.array([[fx, 0, cx],
      [0, fy, cy],
      [0, 0, 1]])
h, w = 760, 1280
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))


mod = cv2.STITCHER_PANORAMA
if int(cv2.__version__[0]) == 3 :
   stitcher = cv2.createStitcher(mod)
else :
   stitcher = cv2.Stitcher_create(mod)

class ImageSubscriber(Node) :
   def __init__(self) :
     super().__init__('image_sub')
     self.publisher = self.create_publisher(Image, 'stitched', 10)
     time_period = 0.5
     self.timer = self.create_timer(time_period, self.timer_callback)
     self.stitched_image = None
     qos = QoSProfile(depth=10)
     self.cam0_sub = self.create_subscription(
	Image,
	'/usb_cam_00/image_raw',
	self.cam0_callback,
	qos)
     self.cam1_sub = self.create_subscription(
	Image,
	'/usb_cam_01/image_raw',
	self.cam1_callback,
	qos)
     self.cam0 = np.empty(shape=[1])
     self.cam1 = np.empty(shape=[1])

   def cam0_callback(self, data) :
     self.cam0 = bridge.imgmsg_to_cv2(data, 'bgr8')
     #print("cam0:",self.cam0.shape)
     #undistorted_image = self.undistort(self.image)

   def cam1_callback(self, data) :
     self.cam1 = bridge.imgmsg_to_cv2(data, 'bgr8')
     # print("cam1:",self.cam0.shape, "cam2:", self.cam1.shape)
     self.cam0 = self.undistort_pinhole(self.cam0)
     self.cam1 = self.undistort_pinhole(self.cam1)
     self.cam0 = cv2.resize(self.cam0, (640, 360))
     self.cam1 = cv2.resize(self.cam1, (640, 360))
     #if self.cam0.shape == self.cam1.shape :
     #  images = cv2.hconcat([self.cam0, self.cam1])
     cv2.imshow('img1', self.cam0)
     cv2.imshow('img2', self.cam1)
     #  cv2.imshow('images', images)

     stitched_image = self.image_stitching([self.cam0, self.cam1])
     self.stitched_image = stitched_image
     if stitched_image.shape[0]> 0 and stitched_image.shape[0] > 0 :
     #  stitched_image = cv2.resize(stitched_image, (1280, 360))
       cv2.imshow('stitched', stitched_image)
     cv2.waitKey(33)

   def timer_callback(self) : 
     image = self.stitched_image 
     if image is not None :
        try :
          img_msg = bridge.cv2_to_imgmsg(image)
          self.publisher.publish(img_msg)
          self.get_logger().info('Image Published')
        except :
          pass
        

   def undistort_pinhole(self, image) :
     dst = cv2.undistort(image, mtx, dist, None, newcameramtx)
     x, y, w, h = roi
     dst = dst[y:y+h, x:x+w]
     return dst 

   def undistort_fisheye(self, image, balance=0.0, dim2=None, dim3=None) :
     dim1 = image.shape[:2][::-1]
     if not dim2 :
       dim2 = dim1
     if not dim3 :
       dim3 = dim1

     scaled_K = K*dim1[0]/DIM[0]
     scaled_K[2][2] = 1.0
     new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=0.0)
     map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
     undistorted_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
     return undistorted_image
     
   def image_stitching(self, image_list, stitched_copy=np.empty(shape=[1])) :
     try :
       status, stitched = stitcher.stitch(image_list)
       if status != 0 :
         print(f'process failed : {status}')
       else :
         gray = cv2.cvtColor(stitched, cv2.COLOR_BGR2GRAY)
         threshold = cv2.bitwise_not(cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1])
         threshold = cv2.medianBlur(threshold, 5)
         stitched_copy = stitched.copy()  
         threshold_copy = threshold.copy()

       while np.sum(threshold_copy) > 0 :
         threshold_copy = threshold_copy[1:-1, 1:-1]
         stitched_copy = stitched_copy[1:-1, 1:-1]
     except :
       pass
     finally :
       return stitched_copy

def main(args=None) :
  rclpy.init(args=args)
  node = ImageSubscriber()

  try :
    rclpy.spin(node)
  except KeyboardInterrupt :
    node.get_logger().info('Stopped by Keyboard')
  finally :
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__' :
  main()
