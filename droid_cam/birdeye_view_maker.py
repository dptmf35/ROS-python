#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()

p1 = [200, 300]
p2 = [400, 300]
p3 = [500, 480]
p4 = [100, 480]

src = np.float32([p1, p2, p3, p4])



class camPublisher(Node) :
  def __init__(self) :
    super().__init__('cam_publisher')
    self.publisher = self.create_publisher(Image, 'droid_cam', 10)
    time_period = 0.01
    self.timer = self.create_timer(time_period, self.time_callback)
    self.cap = cv2.VideoCapture('http://172.30.1.39:4747/video')

  def time_callback(self) :
    ret, frame = self.cap.read()
    h, w = frame.shape[:2]
    image_p1 = [0, 0] 
    image_p2 = [w, 0]
    image_p3 = [w, h]
    image_p4 = [0, h]
    dst = np.float32([image_p1, image_p2, image_p3, image_p4])
    mat = cv2.getPerspectiveTransform(src, dst)
    img_transformed = cv2.warpPerspective(frame, mat, (w, h))
    image = cv2.hconcat([frame, img_transformed])

    if ret == True :
      fra = bridge.cv2_to_imgmsg(frame)
      self.publisher.publish(fra)
      cv2.imshow('droidcamframe', image)
      cv2.waitKey(2)
    self.get_logger().info('Publishing Droidcam Image')


def main(args=None) :
  rclpy.init(args=args)
  node = camPublisher()
  try :
    rclpy.spin(node)
  except KeyboardInterrupt :
    node.get_logger().info('Publish Stopped')
  finally :
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
  main()
