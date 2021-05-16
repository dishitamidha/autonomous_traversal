#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix, Imu, LaserScan, Range
import pyproj
from tf.transformations import euler_from_quaternion
import numpy as np
from geometry_msgs.msg import Twist
import math 
import time

# defining a class
class Robot:
    def __init__(self):
        self.lat = 0   
        self.long = 0  
        self.lat_goal =  49.9000664112   
        self.long_goal =  8.90000796191   
        self.yaw = 0
        self.distance = 0
        self.right = 0
        self.left = 0
        self.az = 0 
        self.linear = False
        self.status = 0
        self.regions = 0
        self.rate = rospy.Rate(4)
        rospy.Subscriber('/group_task2/fix', NavSatFix, self.calc_geod) # to get gps coordinates
        rospy.Subscriber('/group_task2/imu', Imu, self.calc_yaw)  # to get pose from imu
        rospy.Subscriber('/group_task2/laser/scan', LaserScan, self.laser_regions) # to get range array from Hokuyo Lidar
        rospy.Subscriber('/sensor/ir_right', Range, self.right_ir) # to get range data from right ultrasonic
        rospy.Subscriber('/sensor/ir_left', Range, self.left_ir)  # to get range data from left ultrasonic
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) # to publish speed values to cmd_vel topic
        self.rate.sleep()

# calculating distance to be travelled
    def calc_geod(self, data1):
        self.lat = data1.latitude
        self.long = data1.longitude
        g = pyproj.Geod(ellps='WGS84')
        x = g.inv(self.long, self.lat, self.long_goal, self.lat_goal)
        self.az = x[0]
        self.az = self.change_range(self.az)
        self.distance = x[2]
      
# calculating angle by which to move to orient itself towards goal
    def calc_yaw(self, data2):
        orient = data2.orientation
        orient_array = np.array([orient.x, orient.y, orient.z, orient.w]) 
        orient_array_euler = euler_from_quaternion(orient_array)
        self.yaw = orient_array_euler[2]
        self.yaw = self.yaw * 180 /math.pi
        self.yaw = -self.yaw
        self.yaw = self.change_range(self.yaw)
    
# dividing samples of lidar into 3 regions : right, left, front    
    def laser_regions(self, data3):
        self.regions = {
        'right':  min(min(data3.ranges[0:240]), 10),
        'front':  min(min(data3.ranges[241:480]), 10),
        'left':   min(min(data3.ranges[480:719]), 10),
    }
# callback to extract daata from ultrasonic on right side    
    def right_ir(self, data4):
        self.right = data4.range
# callback to extract daata from ultrasonic on right side     
    def left_ir(self, data5):
        self.left = data5.range
# to change the range of angles from (-180 to 180) to (0, 360) for easier calculations
    def change_range(self, y):
        if y<0:
            y = y + 360 
            return y
        else: 
            return y
# function from publishing speeds to cmd_vel topic     
    def publish_vel(self, linear_x, angular_z):
        vel = Twist()
        vel.linear.x = linear_x
        vel.angular.z = angular_z
        self.pub.publish(vel)
# to avoid collision on the side incase rover gets to close to the obstacle in a parallel manner    
    def avoid_side_collision(self):
        vel = Twist()
        while self.right < 0.5:
            rospy.loginfo('Avoiding side collision')
            self.publish_vel(0.2, -0.5)
            if self.right < 0.1:
                rospy.loginfo("eww")
                self.publish_vel(0, -0.5)

        while self.left < 0.5:
            rospy.loginfo('Avoiding side collision')
            self.publish_vel(0.2, 0.5)
            if self.left < 0.1:
                rospy.loginfo("eww")
                self.publish_vel(0, 0.5)
# function for aligning the rover towards goal 
    def orientation_rover(self, angle):
        vel = Twist()
        while not rospy.is_shutdown(): 
            if self.regions['left'] < 0.5 or self.regions['front'] < 0.75 or self.regions['right'] < 0.5:  # calling obstacle avoidance
                self.obstacle_avoidance()                                                                  # incase of obstacle while orienting 
            self.avoid_side_collision()  
            
            if angle > self.yaw:
                if angle-self.yaw > 180:
                    self.publish_vel(0.2, -0.8)
                    rospy.loginfo('case 1')
                
                elif angle-self.yaw < 180:
                    if angle-self.yaw <=0.9:
                        self.linear = True
                        self.publish_vel(0, 0)
                        print "end"
                        break
                    else:
                        self.publish_vel(0.2, 0.8)
                        rospy.loginfo('case 2')
                       
            elif self.yaw > angle:
                if self.yaw-angle > 180:
                    self.publish_vel(0.2, 0.8)
                    rospy.loginfo('case 3')
                  
                elif self.yaw-angle < 180:
                    if self.yaw-angle<=0.9: #1
                        self.linear = True
                        self.publish_vel(0, 0)
                        print "end"
                        break
                    else:
                        self.publish_vel(0.2, -0.8)
                        rospy.loginfo('case 4')
    
 # to traverse linearly after orienting towards goal   
    def linear_rover(self):
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        vel = Twist()
        while not rospy.is_shutdown():
            if(self.distance>0.9): #0.8
                self.publish_vel(0.7, 0)
                
                if self.regions['left'] < 1.5 or self.regions['front'] < 1.5 or self.regions['right'] < 1.5: # to avoid obstacles 
                    self.obstacle_avoidance()                                                                # incase they are in the way
                self.avoid_side_collision()
                    
                if abs(self.az-self.yaw)>2:
                    self.orientation_rover(self.az)
                
            else:
                vel.linear.x = 0
                self.pub.publish(vel)
                break
            print "Distance from goal: ", self.distance

# function for obstacle avoidance
    def obstacle_avoidance(self):
        vel = Twist()
    
        while self.regions['left'] < 1.5 and self.regions['front'] > 1.5 and self.regions['right'] > 1.5:
            rospy.loginfo("Case 1 - Avoid left")
            self.publish_vel(0.5, 0.5)
           
            while self.left < 0.5:
                rospy.loginfo("[case 1 - Avoid left] avoiding narrow collision")
                self.publish_vel(0.2, 0.5)
               
                while self.left < 0.3:
                    rospy.loginfo("[case 1 - Avoid left] avoiding very narrow collision")
                    self.publish_vel(0, 0.5) #
                 

        while self.regions['left'] > 1.5 and self.regions['front'] > 1.5 and self.regions['right'] < 1.5:
            rospy.loginfo("Case 2 - Avoid right")
            self.publish_vel(0.5, -0.5)

            while self.right < 0.5:
                rospy.loginfo("[case 2 - Avoid right] avoiding narrow collision")
                self.publish_vel(0.2, -0.5)
              
                while self.right < 0.3:
                    rospy.loginfo("[case 2 - Avoid right] avoiding very narrow collision")
                    self.publish_vel(0, -0.5) #

  
        while self.regions['left'] > 1.5 and self.regions['front'] < 1.5 and self.regions['right'] > 1.5:
            rospy.loginfo("Case 3 - Avoid front")
            self.publish_vel(0, -0.5)
           
            while self.regions['left'] > self.regions['right']:
                self.publish_vel(0.5, -0.5)

                while self.regions['front'] < 0.75:
                    rospy.loginfo('[case 3 - Avoid front] protecting from front')
                    self.publish_vel(0, -0.8)
                
                while self.right < 0.5:
                    rospy.loginfo("Case 3 - Avoid front - right")
                    self.publish_vel(0.2, -0.5)
                 
                    if self.right < 0.3 or self.regions['front'] < 0.5:
                        self.publish_vel(0, -0.5) #
               
              
            while self.regions['left'] < self.regions['right']:
                self.publish_vel(0.5, 0.5)
                
                while self.regions['front'] < 0.75:
                    rospy.loginfo('[case 3 - Avoid front] protecting from front')
                    self.publish_vel(0, 0.8)

                while self.left < 0.5:
                    rospy.loginfo("Case 3 - Avoid front - left")
                    self.publish_vel(0.2, 0.5)

                    if self.left < 0.3 or self.regions['front'] < 0.5:
                        self.publish_vel(0, 0.5) #
                    

        while self.regions['left'] < 1.5 and self.regions['front'] < 1.5 and self.regions['right'] > 1.5:
            rospy.loginfo("Case 4 - Avoid frontleft")
            self.publish_vel(0.5, 0.5)
           
            while self.regions['front'] < 0.5 or self.regions['left'] < 0.5:
                rospy.loginfo('[case 4 - Avoid frontleft] protecting from front')
                self.publish_vel(0, 0.8)
               
            while self.left < 0.5:
                rospy.loginfo("[case 4 - Avoid frontleft] avoiding narrow collision !!!")
                self.publish_vel(0.5, 0.5)
             
                if self.left < 0.3 or self.regions['front'] < 0.5 or self.regions['left'] < 0.5:
                    self.publish_vel(0, 0.5) #
                   
       
        while self.regions['left'] > 1.5 and self.regions['front'] < 1.5 and self.regions['right'] < 1.5:
            rospy.loginfo("Case 5 - Avoid frontright")
            self.publish_vel(0.5, -0.5)
       
            while self.regions['front'] < 0.5 or self.regions['right'] < 0.5:
                rospy.loginfo('[case 5 - Avoid frontright] protecting from front')
                self.publish_vel(0, -0.8)
                
            while self.right < 0.5:
                rospy.loginfo("[case 5 - Avoid frontright] avoiding narrow collision !!!")
                self.publish_vel(0.5, -0.5)
             
                if self.right < 0.3 or self.regions['front'] < 0.5 or self.regions['right'] < 0.5:
                    self.publish_vel(0, -0.5) #
                   

        while self.regions['left'] < 1.5 and self.regions['front'] > 1.5 and self.regions['right'] < 1.5:
            rospy.loginfo('case 6 - Left and right blocked')
            vel.linear.x = 0.5
            self.pub.publish(vel)
            while self.right < 1.5 and self.left > 1.5:
                rospy.loginfo('[case 6 - Left and right blocked] avoiding narrow - right')
                self.publish_vel(0.5, -0.5)
            
            while self.left < 1.5 and self.right > 1.5:
                rospy.loginfo('[case 6 - Left and right blocked] avoiding narrow - left')
                self.publish_vel(0.5, 0.5)
   
            while self.left < 1.5 and self.right < 1.5:
                rospy.loginfo('[case 6 - Left and right blocked] narrow path - being careful!')
                
                if self.left > self.right:
                    self.publish_vel(0.5, -0.5)
                  
                else: 
                    self.publish_vel(0.5, 0.5)
              
        
        while self.regions['left'] < 1 and self.regions['front'] < 1 and self.regions['right'] < 1:
            rospy.loginfo("Case 7 - All blocked")

            
            while self.regions['front'] < 1 and self.regions['right'] < 1 and self.regions['left'] - self.regions['right'] < 0.3:
                rospy.loginfo("[Case 7 - All blocked] case 1")
                while self.regions['front'] < 0.5:
                    rospy.loginfo("[Case 7 - All blocked] case 1 protecting from front")
                    self.publish_vel(0, 0.8)
                while self.left < 0.75 and self.regions['front'] > 0.5:
                    rospy.loginfo("[Case 7 - All blocked] case 1 protecting from sides")
                    self.publish_vel(0.5, 0.8)
                self.publish_vel(0.2, 0.8)

            while self.regions['front'] < 1 and self.regions['right'] < 1 and self.regions['right'] - self.regions['left'] < 0.3 :
                rospy.loginfo("[Case 7 - All blocked] case 2")
                while self.regions['front'] < 0.5:
                    rospy.loginfo("[Case 7 - All blocked] case 2 protecting from front")
                    self.publish_vel(0, -0.8)
                while self.right < 0.75 and self.regions['front'] > 0.75:
                    rospy.loginfo("[Case 7 - All blocked] case 2 protecting from side")
                    self.publish_vel(0.5, -0.8)

                self.publish_vel(0.2, -0.8)

# main function to call for orientation and linear movement
    def trav(self):
        self.orientation_rover(self.az)
        while self.linear:
            self.linear_rover()
        rospy.spin()   

if __name__ == "__main__":
    rospy.init_node('listener')
    robot = Robot()
    robot.trav()
    rospy.spin()