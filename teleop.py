import sys
from PyQt6.QtWidgets import QApplication,QGridLayout,QMainWindow, QLabel,QLineEdit, QWidget,QPushButton,QHBoxLayout, QInputDialog
from PyQt6.QtCore import QTimer
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from enum import Enum
import threading
import math

class Direction(Enum):
    LEFT = 0
    RIGHT = 1
    UP = 2
    DOWN = 3

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Teleop")
        self.setGeometry(200,100,500,300)
        self.namespace = ""
        self.teleop_node = teleop_gui()
        self.dir = 1
        self.timer = QTimer()
        
        left = QPushButton("\u2190")
        right = QPushButton("\u2192")
        up = QPushButton("\u2191")
        down = QPushButton("\u2193")
      
        namespace_lbl= QLabel("Enter namespace:")
        velocity_lbl = QLabel("Enter Linear Speed:")
        self.input = QLineEdit()
        self.velInput = QLineEdit()
       
        left.pressed.connect(lambda: self.onPressed(1))
        left.released.connect(lambda: self.onRelease())
        up.pressed.connect(lambda: self.onPressed(2))
        up.released.connect(lambda: self.onRelease())
        right.pressed.connect(lambda: self.onPressed(3))
        right.released.connect(lambda: self.onRelease())
        down.pressed.connect(lambda: self.onPressed(4))
        down.released.connect(lambda: self.onRelease())
        self.velInput.setFixedWidth(40)
        layout = QGridLayout()
        layout.setRowStretch(0,50)
        layout.setRowStretch(1,100)
        layout.setRowStretch(2,100)
        layout.setRowStretch(3,100)
        layout.addWidget(left,2,0)
        layout.addWidget(up,1,1)
        layout.addWidget(right,2,2)
        layout.addWidget(down,3,1)
        layout.addWidget(namespace_lbl,0,4)
        layout.addWidget(self.input,1,4)
        layout.addWidget(velocity_lbl,2,4)
        layout.addWidget(self.velInput,3,4)
        layout.setHorizontalSpacing(10)
        self.timer.timeout.connect(self.moveAction)
        self.setLayout(layout)
    def onPressed(self,dir):
        self.dir = dir
        self.timer.start(500)
        
    def onRelease(self):
        self.timer.stop() 
    def moveAction(self):
        namespace = self.input.text()
        try:
            speed = float(self.velInput.text())
            print("moving in direction: "+str(self.dir))
            self.teleop_node.broadcast(self.dir,namespace,speed)
        except: 
            speed = 0.0
            print('invalid input')
        
  
class teleop_gui(Node):
    def __init__(self):
        super().__init__('teleop_gui')
        self.published_namespaces = []
        self.active_publishers = []
    def broadcast(self, dir,namespace,vel):
        if namespace in self.active_publishers:
            i = self.published_namespaces.index(namespace)
            publisher = self.active_publishers[i]
        else:
            publisher = self.create_publisher(Twist, '/'+namespace+'/cmd_vel',1)
            self.active_publishers.append(publisher)
            self.published_namespaces.append(namespace)
        msg = Twist()
        if dir == 1:
            msg.angular.z = math.pi/2
        if dir == 2:
            msg.angular.z = 0.0
        if dir == 3:
            msg.angular.z = -math.pi/2
        if dir == 4:
            msg.angular.z = math.pi
        msg.linear.x = vel
        publisher.publish(msg)
        #publisher.destroy()
def main(args=None):
    
    app = QApplication(sys.argv)
    app.setStyleSheet('.QPushButton {font-size: 20px}')
    rclpy.init(args = None)
    window = MainWindow()
    node = window.teleop_node
    thread = threading.Thread(target=rclpy.spin,args =(node,),daemon=True)
    
    thread.start()  
    window.show()
    app.exec()
    node.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__=="__main__":
    main()


