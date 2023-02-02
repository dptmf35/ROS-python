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
mod = cv2.STITCHER_PANORAMA
if int(cv2.__version__[0]) == 3 :
  stitcher = cv2.createStitcher(mod)
else :
  stitcher = cv2.Stitcher_create(mod)



class camPublisher(Node) :
  def __init__(self) :
    super().__init__('cam_publisher')
    self.publisher1 = self.create_publisher(Image, 'droid_cam1', 10)
    self.publisher2 = self.create_publisher(Image, 'droid_cam2', 10)
    time_period = 0.01
    self.timer1 = self.create_timer(time_period, self.time_callback1)
    self.timer2 = self.create_timer(time_period, self.time_callback2)
    self.cap1 = cv2.VideoCapture('http://172.30.1.39:4747/video')
    self.cap2 = cv2.VideoCapture('http://172.30.1.41:4747/video')
    self.frame1 = None
    self.frame2 = None

  def time_callback1(self) :
    ret, frame = self.cap1.read()
    h, w = frame.shape[:2]
    # print(h, w)
    image_p1 = [0, 0] 
    image_p2 = [w, 0]
    image_p3 = [w, h]
    image_p4 = [0, h]
    dst = np.float32([image_p1, image_p2, image_p3, image_p4])
    mat = cv2.getPerspectiveTransform(src, dst)
    img_transformed = cv2.warpPerspective(frame, mat, (w, h))
    #image = cv2.hconcat([frame, img_transformed])
    self.frame1 = img_transformed
    if ret == True :
      fra = bridge.cv2_to_imgmsg(frame)
      self.publisher1.publish(fra)
      #cv2.imshow('droidcamframe1', img_transformed)
      #cv2.waitKey(2)
    self.get_logger().info('Publishing Droidcam1 Image')

  def time_callback2(self) :
    ret, frame = self.cap2.read()
    h, w = frame.shape[:2]
    # print(h, w)
    image_p1 = [0, 0]
    image_p2 = [w, 0]
    image_p3 = [w, h]
    image_p4 = [0, h]
    dst = np.float32([image_p1, image_p2, image_p3, image_p4])
    mat = cv2.getPerspectiveTransform(src, dst)
    img_transformed = cv2.warpPerspective(frame, mat, (w, h))
    self.frame2 = img_transformed
    if ret == True :
       fra = bridge.cv2_to_imgmsg(frame)
       self.publisher2.publish(fra)
       images = cv2.hconcat([self.frame1, self.frame2])
       cv2.imshow('droidcams', images)
       cv2.waitKey(2)
    self.get_logger().info('Publishing Droidcam2 Image')
    images = [self.frame1, self.frame2]
    try :
      status, stitched = stitcher.stitch(images)
      if status == 0 :
         cv2.imshow('droidcamstitched', stitched)
         cv2.waitKey(2)
      else :
         pass
    except :
      pass

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
