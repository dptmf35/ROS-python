
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic

#import cv2
#from cv_bridge import CvBridge
import rclpy


form_class = uic.loadUiType('test.ui')[0]


class WindowClass(QMainWindow, form_class) :
   def __init__(self) :
        super().__init__()
        self.setupUi(self)

        self.load.clicked.connect(self.loadImageFromFile)

   def loadImageFromFile(self) :
        self.qPixmapFileVar = QPixmap()
        self.qPixmapFileVar.load("testImage.jpg")
        self.qPixmapFileVar = self.qPixmapFileVar.scaledToWidth(600)
        self.label.setPixmap(self.qPixmapFileVar)


if __name__ == "__main__" :
    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()
    app.exec_()
