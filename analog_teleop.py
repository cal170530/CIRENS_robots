import sys
from PyQt6.QtWidgets import QApplication,QGridLayout,QDial,QMainWindow, QLabel,QLineEdit, QWidget,QPushButton, QInputDialog
from PyQt6.QtCore import QTimer
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QKeyEvent
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from enum import Enum
import threading
import math


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Teleop")
        self.setGeometry(200,100,500,300)
        self.namespace = ""
        self.teleop_node = teleop_gui()
        self.dir = 1
        self.timer = QTimer()
        
        
        self.dial = QDial()
        self.dial.setRange(-180,180)
        self.dial.setSingleStep(1)
        
        namespace_lbl= QLabel("Enter namespace:")
        velocity_lbl = QLabel("Enter Linear Velocity:")
        angVelLbl = QLabel("Enter Angular Velocity:")
        dialLbl= QLabel("    Hold spacebar to move.\n"+"Rotate dial to change direction.")
        self.input = QLineEdit()
        self.velInput = QLineEdit()
        self.angVelInput = QLineEdit()
       
      
        self.dial.sliderMoved.connect(self.getAngle)
        
        self.velInput.setFixedWidth(40)
        self.angVelInput.setFixedWidth(40)

        layout = QGridLayout()
       
        #layout.setColumnStretch(4,100)
        #layout.setRowStretch(3,100)
     
        layout.addWidget(self.dial,2,0,3,3)
        layout.addWidget(dialLbl,0,0)
        layout.addWidget(namespace_lbl,0,4)
        layout.addWidget(self.input,1,4)
        layout.addWidget(velocity_lbl,2,4)
        layout.addWidget(self.velInput,3,4)
        layout.addWidget(angVelLbl,4,4)
        layout.addWidget(self.angVelInput,5,4)
        layout.setHorizontalSpacing(10)
        self.timer.timeout.connect(self.moveAction)
        self.setLayout(layout)
    def keyPressEvent(self, event):
       key = event.key()
       if key == 32 and not event.isAutoRepeat():
           self.timer.start(500)
       

    def keyReleaseEvent(self, event):
        key = event.key()
        if key == 32 and not event.isAutoRepeat():
            print('key released')
            self.timer.stop()
      
    def moveAction(self):
        namespace = self.input.text()
        try:
            linear_vel = float(self.velInput.text())
            angular_vel = float(self.angVelInput.text())
            print("moving in direction: "+str(self.dir))
            self.teleop_node.broadcast(self.dir,namespace,linear_vel,angular_vel)
        except: 
            speed = 0.0
            print('invalid input')
    def getAngle(self):
        self.dir = self.dial.value()
        
  
class teleop_gui(Node):
    def __init__(self):
        super().__init__('teleop_gui')
        self.published_namespaces = []
        self.active_publishers = []
    def broadcast(self, dir,namespace,linear_vel,angular_vel):
        if namespace in self.active_publishers:
            i = self.published_namespaces.index(namespace)
            publisher = self.active_publishers[i]
        else:
            publisher = self.create_publisher(Twist, '/'+namespace+'/cmd_vel',1)
            self.active_publishers.append(publisher)
            self.published_namespaces.append(namespace)
        msg = Twist()
        
        msg.angular.z = -angular_vel*dir/360*2*math.pi
      
        msg.linear.x = linear_vel
        publisher.publish(msg)
        
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


