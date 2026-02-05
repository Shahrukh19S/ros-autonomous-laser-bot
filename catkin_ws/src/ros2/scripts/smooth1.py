#!/usr/bin/env python  
 
from geometry_msgs.msg import Twist
import math
pi=3.1415926535897
 
class SmoothController1:
    def __init__(self, goalIx, goalIy):
        self.goalIx=goalIx
        self.goalIy=goalIy
        
     
    def get_twist(self, x, y, theta):
        vel_msg=Twist()
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
            angle=thetades-theta
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
        
            
            
     
