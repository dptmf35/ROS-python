#!/usr/bin/env python
import sys
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import cv2
from cv_bridge import CvBridge
from PyQt5.QtWidgets import *

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
import numpy as np

#form_class = uic.loadUiType("test.ui")[0]

class ImageSubscriber(Node) :
    #imageChanged = pyqtSignal(QImage)
    def __init__(self) :
       super().__init__('image_sub')
       self.bridge = CvBridge()
       self.qos = QoSProfile(depth=10)
       self.subscriber = self.create_subscription(
	Image,
	'/image',
	self.callback,
	self.qos)
       self.sub_img = None

    def callback(self, data) :
       #print('callback')
       try :
         cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
         #print(cv_image)
       except :
         print(e)
       else: 
         src = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
         h, w, ch = src.shape
         bytesPerLine = ch * w
         qImage = QImage(src.data, w, h, bytesPerLine, 
			QImage.Format_RGB888)
         self.sub_img = qImage
         #self.imageChanged.emit(qImage)
    

class SubImageManager(QObject) :
    imageChanged = pyqtSignal(QImage)

    def __init__(self, parent=None) :
       super(SubImageManager, self).__init__()
       self.image= np.empty(shape=[1])

    def start(self) :
       print('---start')
       node = ImageSubscriber()
       rclpy.spin_once(node)
       self.image = node.sub_img
       self.imageChanged.emit(self.image)
       #self.stop()
       
    def stop(self) :
       if self.subscriber is None :
         
         rclpy.shutdown()
         # self.subscriber = None

class ImageWidget(QWidget) :
    def __init__(self) :
        super().__init__()
        self.button = QPushButton('show picture')
        self.image_frame = QLabel()

        lay = QVBoxLayout(self)
        lay.addWidget(self.button)
        lay.addWidget(self.image_frame)

        self.subscribe_manager = SubImageManager()
        self.subscribe_manager.imageChanged.connect(self.on_image_changed)
        self.button.clicked.connect(self.start)
        
    def start(self) :
        self.subscribe_manager.start()

    @pyqtSlot(QImage) 
    def on_image_changed(self, image) :
       self.qPixVar = QPixmap()
       self.image_frame.setPixmap(QPixmap.fromImage(image))

if __name__ == "__main__" :
    app = QApplication(sys.argv)
    
    rclpy.init(args=None)

    print('rclpy init')
    myWindow = ImageWidget()
    myWindow.show()
    sys.exit(app.exec_())
