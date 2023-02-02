import cv2
import threading
import sys
from PyQt5 import QtWidgets
from PyQt5 import QtGui
from PyQt5 import QtCore

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

bridge= CvBridge()

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
     self.image = bridge.imgmsg_to_cv2(data,'bgr8')
     

running = False
def run():
    global running
    rclpy.init(args=None)
    node = ImageSubscriber()
    while running:
        rclpy.spin_once(node)

        if node :
            img = cv2.cvtColor(node.image, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (0, 0), fx=0.5, fy=0.5) 
            h,w,c = img.shape
            print(img.shape)
            qImg = QtGui.QImage(img.data, w, h, w*c, QtGui.QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(qImg)
            label.setPixmap(pixmap)
        else:
            QtWidgets.QMessageBox.about(win, "Error", "Cannot read frame.")
            print("cannot read frame.")
            break
    print("Thread end.")

def stop():
    global running
    running = False
    print("stoped..")

def start():
    global running
    running = True
    th = threading.Thread(target=run)
    th.start()
    print("started..")

def onExit():
    print("exit")
    stop()

app = QtWidgets.QApplication([])
win = QtWidgets.QWidget()
vbox = QtWidgets.QVBoxLayout()
label = QtWidgets.QLabel()
btn_start = QtWidgets.QPushButton("START")
btn_stop = QtWidgets.QPushButton("END")
vbox.addWidget(label)
vbox.addWidget(btn_start)
vbox.addWidget(btn_stop)
win.setLayout(vbox)
win.show()

btn_start.clicked.connect(start)
btn_stop.clicked.connect(stop)
app.aboutToQuit.connect(onExit)

sys.exit(app.exec_())
