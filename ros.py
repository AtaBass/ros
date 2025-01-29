#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import sqrt
import tf
from tf.transformations import euler_from_quaternion

class Turtlebot3Controller:
    def __init__(self):
        rospy.init_node('turtlebot3_controller', anonymous=True)
        
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
       
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        
        self.current_position = Point()
        
        
        self.target_position = Point()
        self.target_position.x = 1.0 
        self.target_position.y = 1.0  
        
        self.linear_speed = 0.2  
        self.distance_threshold = 0.1 
        
        self.rate = rospy.Rate(10) 
    
    def odom_callback(self, msg):
       
        self.current_position = msg.pose.pose.position
    
    def get_distance_to_target(self):
        
        return sqrt(
            (self.target_position.x - self.current_position.x) ** 2 +
            (self.target_position.y - self.current_position.y) ** 2
        )
    
    def move_to_target(self):
        while not rospy.is_shutdown():
            
            distance = self.get_distance_to_target()
            
           
            vel_msg = Twist()
            
            if distance > self.distance_threshold:
               
                vel_msg.linear.x = min(self.linear_speed, distance)
            else:
               
                vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(vel_msg)
                rospy.loginfo("Target reached!")
                break
            
           
            self.cmd_vel_pub.publish(vel_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = Turtlebot3Controller()
        controller.move_to_target()
    except rospy.ROSInterruptException:
        pass