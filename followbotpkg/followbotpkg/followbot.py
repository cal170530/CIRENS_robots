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

from turtlebot4_msgs.msg import UserLed

from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# FollowBot state


class FollowBot(Node):

    is_docked = False

    target = None

    def __init__(self,agent,leader):
        super().__init__('followbot')
        tf_topic  = '/'+leader+'/tf'
        self.agent = agent
        self.leader= leader
      
        dock_sub = self.create_subscription(DockStatus,
                                            '/dock',
                                            self.dockCallback,
                                            qos_profile_sensor_data)
 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
   
        self.timer = self.create_timer(1.0,self.on_timer)
        self.publisher = self.create_publisher(Twist, '/robot2/cmd_vel',1)

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

    # Send cmd_vel to Create 3
    def drive(self, linear_x, angular_z):
        msg = Twist()
        msg.angular.z = angular_z
        msg.linear.x = linear_x

        self.publisher.publish(msg)

    # Undock action
    def undock(self):
        self.undock_action_client.wait_for_server()
        undock_goal_result = self.undock_action_client.send_goal(Undock.Goal())
        if undock_goal_result.result.is_docked:
            print('Undocking failed')

    # Calculate direction of leader relative to agent

    def on_timer(self):
      
        try:
            t= self.tf_buffer.lookup_transform(self.agent,
                                               self.leader,
                                               rclpy.time.Time())
            print(self.leader+'-'+self.agent+': '+'x:'+str(t.transform.translation.x) +' y:'+ str(t.transform.translation.y))
           
            scale_rotation_rate = 1.0
            rot_angle = scale_rotation_rate*math.atan2(
                t.transform.translation.y,
                t.transform.translation.x
            )
            scale_forward_speed = 0.5
            drive_distance = scale_forward_speed*math.sqrt(
                t.transform.translation.x**2+
                t.transform.translation.y**2
            )
            self.drive(drive_distance,rot_angle)
            print(self.agent+' following '+self.leader)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.agent} to {self.leader}: {ex}')
            return
 

