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
#import numpy as np
import math
from geometry_msgs.msg import TransformStamped
from turtlebot4_msgs.msg import UserLed
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist

from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf2_msgs.msg import TFMessage


# FollowBot state

# def quaternion_from_euler(ai, aj, ak):
#     ai /= 2.0
#     aj /= 2.0
#     ak /= 2.0
#     ci = math.cos(ai)
#     si = math.sin(ai)
#     cj = math.cos(aj)
#     sj = math.sin(aj)
#     ck = math.cos(ak)
#     sk = math.sin(ak)
#     cc = ci*ck
#     cs = ci*sk
#     sc = si*ck
#     ss = si*sk

#     q = np.empty((4, ))
#     q[0] = cj*sc - sj*cs
#     q[1] = cj*ss + sj*cc
#     q[2] = cj*cs - sj*sc
#     q[3] = cj*cc + sj*ss

#     return q

class LeaderBot(Node):

    def __init__(self):
        super().__init__('leaderbot')
        tf_topic  = '/robot1/tf'
       
  
        self.frame = self.declare_parameter(
           'leader_frame', '/robot1/base_link').get_parameter_value().string_value
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
             msg_type= TFMessage,
             topic = tf_topic,
             callback=self.tf_callback,
              qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE, depth=10)
        )  
        
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


    def tf_callback(self,msg):
      
        for t in msg.transforms:
            if t.header.frame_id == '/robot1/odom':
              self.tf_broadcaster.sendTransform(t)  

    def run(self):
        # Undock first
        if self.is_docked:
            print('Undocking')
            self.undock()

        while True:
           
           print('running')
           time.sleep(3)
           time.sleep(1/self.fps)


