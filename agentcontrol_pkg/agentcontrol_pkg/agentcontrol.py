#!/usr/bin/env python3

# Copyright 2021 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import threading
import time

from enum import Enum

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

import math
import numpy as np
from turtlebot4_msgs.msg import UserLed
from geometry_msgs.msg import Twist

from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Agent State



   
class AgentController(Node):

    def __init__(self,i,neighbors):
        super().__init__('AgentController')
        self.agent = 'robot'+str(i)
        self.id = i
        self.neighbors= neighbors
        #self.X = np.zeros((len(neighbors),2))
        self.X = dict.fromkeys(neighbors)
        self.trackedNeighbors = dict.fromkeys(neighbors)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.rot_vel = 0.5
        self.linear_vel = 1.0
        self.timer = self.create_timer(1.0,self.on_timer)
        self.publisher = self.create_publisher(Twist, '/'+self.agent+'/cmd_vel',1)
        
           
    # Dock subscription callback
    def dockCallback(self, msg: DockStatus):
        self.is_docked = msg.is_docked


    # Set User LEDs for TurtleBot 4
    def setLed(self, led, color, period, duty):
        msg = UserLed()
        msg.led = led
        msg.color = color
        msg.blink_period = period
        msg.duty_cycle = duty

        self.user_led_pub.publish(msg)

    # Undock action
    def undock(self):
        self.undock_action_client.wait_for_server()
        undock_goal_result = self.undock_action_client.send_goal(Undock.Goal())
        if undock_goal_result.result.is_docked:
            print('Undocking failed')

    # Calculate direction of leader relative to agent

    def on_timer(self):
      
        for neighbor in self.neighbors:
            try:
                #i = self.neighbors.index(neighbor)
                
                self.X[neighbor] = np.array((0.0,0.0))
                if neighbor != self.agent:
                    t= self.tf_buffer.lookup_transform(self.agent,
                                                        neighbor,
                                                        rclpy.time.Time())
                    self.X[neighbor] = np.array((t.transform.translation.x, t.transform.translation.y))
             
                
                self.trackedNeighbors[neighbor] = 1
                
            
                
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {self.agent} to {neighbor}: {ex}')
                self.trackedNeighbors[neighbor] = 0
                pass
            self.Controller()
                   

#------------ CONTROLLER
#------------ X[k] is the state of robot{k}
#------------ Xi is the state of the controlled robot
#------------ Xcnext = The desired next state of current robot
#------------
#------------ Consensus Algorithm:
    def Controller(self):
        #Xi = self.X[self.id]
        Xi = np.array((0.0,0.0))
        dx = np.array((0.0,0.0))
       # print('Xi=' +str(type(Xi)))
       # print('dx=' +str(type(dx)))
        count = 0
        for neighbor in self.neighbors:
            #k = self.neighbors.index(neighbor)
            #if k != self.id and self.trackedNeighbors[neighbor] == 1:
         #   print(type(self.X[neighbor]))
            print(neighbor+str(self.X[neighbor]))
          
            if neighbor != self.agent and self.trackedNeighbors[neighbor] ==1:
                Xn = self.X[neighbor]
          #      print('Xn='+str(type(Xn)))
          #      print(str(Xi))
          #      print(str(Xn))
                dx+= Xn-Xi
                count+=1
        if count>0:
             dx = dx/count
             
       
        self.stateUpdate(dx)

#------------STATE UPDATE
#------------Need to determine how best to get a displacement of 'dx' through one twist msg..
#------------Current method: 'rotate then move straight'

    def stateUpdate(self, dx):
        msg = Twist()
        x= dx[0]
        y= dx[1]
        if math.sqrt(dx[0]**2+dx[1]**2)< 0:
            print(self.agent+' has reached consensus.'+str(dx))
        else:
            theta = math.atan2(y,x)
            if theta > 0.2:
                msg.angular.z = self.rot_vel*theta
                msg.linear.x = 0.0
                print('theta'+str(theta))
                self.publisher.publish(msg)
            elif theta <-0.2:
                msg.angular.z = theta
                msg.linear.x = 0.0
                print('theta'+str(theta))
                self.publisher.publish(msg)
            else: 
                print(self.agent+' Moving'+str(x) +str(y))   
                msg.angular.z =self.rot_vel*theta
                msg.linear.x = self.linear_vel*math.sqrt(x**2+y**2)
                self.publisher.publish(msg)
