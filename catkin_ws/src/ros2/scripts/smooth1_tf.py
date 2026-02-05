#!/usr/bin/env python  
import rospy
from geometry_msgs.msg import Twist 
import math
import tf
from math import pi
import numpy as np
from tf.transformations import euler_from_quaternion
#pi=3.1415926535897
 
class SmoothController1TF:
    def __init__(self, listener, goalIx, goalIy):
        self.goalIx=goalIx
        self.goalIy=goalIy
        self.listener=listener
      
        
     
    def get_twist(self):
        vel_msg=Twist()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform('/odom', '/base_footprint', \
                                                       rospy.Time(0))
            except (tf.LookupException, \
                tf.ConnectivityException, tf.ExtrapolationException):
                  continue
            x=trans[0]
            y=trans[1]
            euler = euler_from_quaternion(rot)
            pheta=euler[2]
            pheta=-pheta
            dth=self.goalIx-x
            if dth>0:
                vel_msg.linear.x=abs(0.2)
                vel_msg.linear.y=0
                vel_msg.linear.z=0
                vel_msg.angular.x=0
                vel_msg.angular.y=0
                vel_msg.angular.z=0
                return vel_msg
            else:
                thetades=1.466
                angle=thetades-pheta
                if angle>0:
                    vel_msg.angular.z=-0.4
                else:
                    dty=(y-self.goalIy)
                    if dty>0:
                        vel_msg.linear.x=abs(0.2)
                        vel_msg.linear.y=0
                        vel_msg.linear.z=0
                        vel_msg.angular.x=0
                        vel_msg.angular.y=0
                        vel_msg.angular.z=0
                    else:
                        vel_msg.linear.x=(0)
                        vel_msg.linear.y=0
                        vel_msg.linear.z=0
                        vel_msg.angular.x=0
                        vel_msg.angular.y=0
                        vel_msg.angular.z=0
                return vel_msg
           
                        

          
            rate.sleep()

            
        

        
        
                
      
    
     
            
                
            
                
          
       
                
        
            
            
     
