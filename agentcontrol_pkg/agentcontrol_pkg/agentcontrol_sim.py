


from enum import Enum
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

import math
import numpy as np
from turtlebot4_msgs.msg import UserLed
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, qos_profile_sensor_data
from turtlebot4_msgs.msg import UserLed


#pose.orientation is a quaternion, this converts it to an angle which represents the global heading of the robot
def quaternion_to_heading_angle(q):
    """
    Finds the heading angle of the robot from its quaternion orientation
    :param q: quaternion rotation from ROS msg
    :return:
    """
    return np.arctan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))
   
class AgentController_Sim(Node):

    def __init__(self,i,neighbors,sim = 0):
        super().__init__('AgentController_Sim')
        self.agent = 'robot'+str(i)
        self.id = i
        self.neighbors= neighbors
        self.X = dict.fromkeys(neighbors)
        self.trackedNeighbors = dict.fromkeys(neighbors)        
        self.subscribers = dict.fromkeys(neighbors)  
        self.heading = 0.0  
        self.rot_vel = 0.7
        self.linear_vel = 1.0
        self.user_led_pub = self.create_publisher(UserLed, '/'+self.agent+'/hmi/led', qos_profile_sensor_data)
        self.sim = sim
        if self.sim == 0:
            self.min_prox = 0.35
            self.min_goal_prox = 0.36
        elif self.sim ==1:
            self.min_prox = 0.35
            self.min_goal_prox = 0.36
        self.timer = self.create_timer(0.2,self.on_timer)
        self.publisher = self.create_publisher(Twist, '/'+self.agent+'/cmd_vel',1)
        self.create_subscribers()
    def create_subscribers(self):
        if  self.sim ==0:
            for neighbor in self.neighbors:
                print(self.agent+' has neighbor: '+neighbor)
                try:
                    pose_subscriber = self.create_subscription(
                        Odometry,
                        '/'+neighbor+'/sim_ground_truth_pose',
                        lambda msg,name = neighbor :self.pose_callback(msg,name),
                        #lambda msg, neighbor: self.pose_callback(msg, neighbor),
                        qos_profile_sensor_data)
                    self.subscribers[neighbor] = pose_subscriber
                    print("Adding subscriber:")
                    print("  topic: ", '/'+neighbor+'/sim_ground_truth_pose')
                    
                except:
                    print('no subscription was made.')
        elif self.sim == 1:
                   
            for neighbor in self.neighbors:
                print(self.agent+' has neighbor: '+neighbor)
                try:
                    pose_subscriber = self.create_subscription(
                        PoseStamped,
                        '/vcrn/mocap/'+neighbor+'/pose',
                        lambda msg,name = neighbor :self.pose_callback(msg,name),
                        #lambda msg, neighbor: self.pose_callback(msg, neighbor),
                        qos_profile_sensor_data)
                    self.subscribers[neighbor] = pose_subscriber
                    print("Adding subscriber:")
                    print("  topic: ", '/'+neighbor+'/sim_ground_truth_pose')
                    
                except:
                    print('no subscription was made.')
            
        
    def pose_callback(self, msg,neighbor):
        """ callback function to get the pose from mocap data """
        if self.sim == 0:
            x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        elif self.sim ==1:
            x,y = msg.pose.position.x, msg.pose.position.y
        if neighbor == self.agent:
            if self.sim ==0:
                self.heading = quaternion_to_heading_angle(msg.pose.pose.orientation)
            elif self.sim ==1:
                self.heading = quaternion_to_heading_angle(msg.pose.orientation)
        self.X[neighbor] = np.array((x,y))  
    # Dock subscription callback(Not currently used)
    def dockCallback(self, msg: DockStatus):
        self.is_docked = msg.is_docked


    # Set User LEDs for TurtleBot 4)
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
        tracking = True
        for neighbor in self.neighbors:
            if self.X[neighbor] is None:
                tracking = False
                #print(self.agent +' does not see '+ neighbor)
            else:
                self.trackedNeighbors[neighbor]=1
                #print(self.agent+' sees '+str(self.X[neighbor]))
     

        if tracking == True:
            #print(self.agent+ ' controller is called.....')
            self.Controller()
                   

#------------ CONTROLLER
#------------ X[k] is the state of robot{k}
#------------ Xi is the state of the controlled robot
# #------------ Consensus Algorithm:
    def Controller(self):
        Xi = self.X[self.agent]
        dx = np.array((0.0,0.0))
        count = 0
        avoid_list = []
        for neighbor in self.neighbors:

            if self.trackedNeighbors[neighbor] == 1:
                diff = self.X[neighbor]- Xi
                euclid_diff = math.sqrt(diff[0]**2+diff[1]**2)
                if euclid_diff<= self.min_prox and neighbor != self.agent:
                    print(self.agent+' is '+str(euclid_diff)+' away from '+neighbor)
                    avoid_list.append(diff)
                
                dx+= diff
                count+=1
                #print(dx)
        dx = dx/count
        #if ( not math.isnan(dx[0]) and  not math.isnan(dx[1]) and not diff == [0,0]):
        
        self.stateUpdate(dx,avoid_list)

#------------STATE UPDATE
#------------Need to determine how best to get a displacement of 'dx' through one twist msg..
#------------Current method: 'rotate then move straight'
     
    def stateUpdate(self, dx,avoid_list = []):
        

        msg = Twist()
        x= dx[0]
        y= dx[1]
        dist_from_goal = math.sqrt(x**2+y**2)
        theta = math.atan2(y,x)
        dtheta = theta-self.heading
        if dist_from_goal <self.min_goal_prox:
            self.setLed(0, 1, 500, 0.5)
            print(self.agent+' has reached goal.')
        if dtheta > 0.2:
            msg.angular.z = self.rot_vel*dtheta
            msg.linear.x = 0.0
            #(self.agent+'rotating to '+str(self.rot_vel*dtheta))
        elif dtheta <-0.2:
            msg.angular.z = self.rot_vel*dtheta
            msg.linear.x = 0.0
            #print(self.agent+'rotating to '+str(self.rot_vel*dtheta))
        else:   
            msg.angular.z =self.rot_vel*dtheta
            msg.linear.x = self.linear_vel*min(dist_from_goal,1.5)
        if avoid_list != [] and dist_from_goal> self.min_goal_prox:
            print(self.agent+' avoiding')
            self.reroute(avoid_list)
        elif dist_from_goal> self.min_goal_prox:
            self.publisher.publish(msg)
        
    
    def reroute(self,avoid_list):
    #Untested..
        msg = Twist()
        theta_avoid =[]
        for loc in avoid_list:
            x = avoid_list[0]
            y = avoid_list[1]
            theta = math.atan2(y,x)
            theta_avoid.append(theta-self.heading)
        rightmost = max(theta_avoid)+.3
        msg.angular.z = self.rot_vel*rightmost
        msg.linear.x = 0.2
        self.publisher.publish(msg)
