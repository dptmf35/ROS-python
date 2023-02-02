#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()
DIM=(1920, 1080)
K=np.array([[1041.721033566984, 0.0, 982.4318615455983], [0.0, 1041.0260709694585, 431.8071863472761], [0.0, 0.0, 1.0]])
D=np.array([[-0.07960228346280786], [0.4447533452768294], [-1.6226455570763598], [1.907617662076584]])


class ImageSubscriber(Node) :
   def __init__(self) :
     super().__init__('image_sub')
     qos = QoSProfile(depth=10)
     self.image_sub = self.create_subscription(
	Image,
	'/image',
	self.image_callback,
	qos)
     self.image = np.empty(shape=[1])


   def image_callback(self, data) :
     self.image = bridge.imgmsg_to_cv2(data, 'bgr8')
     print(self.image)
     #undistorted_image = self.undistort(self.image)
     #cv2.imshow('img', self.image)
     #cv2.waitKey(33)
     #cv2.imwrite('./test.jpg', undistorted_image)

   def undistort(self, image, balance=0.0, dim2=None, dim3=None) :
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
      


def main(args=None) :
  print('--init')
  rclpy.init(args=args)
  print('--after init')
  node = ImageSubscriber()
  print('init node')

  try :
    rclpy.spin(node)
  except KeyboardInterrupt :
    node.get_logger().info('Stopped by Keyboard')
  finally :
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__' :
  main()
