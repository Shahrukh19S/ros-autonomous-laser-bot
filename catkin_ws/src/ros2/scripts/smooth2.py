#!/usr/bin/env python  
 
from geometry_msgs.msg import Twist
import math
from math import pi
import numpy as np
#pi=3.1415926535897
 
class SmoothController2:
    def __init__(self, goalIx, goalIy,g0):
        self.goalIx=goalIx
        self.goalIy=goalIy
        self.g0=g0
        
     
    def get_twist(self, x, y, theta):
        vel_msg=Twist()
        K_rho=0.6
        K_alpha=1.6
        K_theta=0.3
        El=np.array([[x],[y],[theta]], dtype=float )
        Gl=np.array([[self.goalIx],[self.goalIy],[self.g0]], dtype=float)
        Rcw=np.array([[math.cos(theta), math.sin(theta), 0], [-math.sin(theta), math.cos(theta), 0], [0, 0, 1]], dtype=float)
        tp=Gl-El
        gr=np.dot(Rcw,tp)
        grx=np.asscalar(gr[0])
        gry=np.asscalar(gr[1])
        gr0=np.asscalar(gr[2])
        rho=math.sqrt(grx**2+gry**2)
        alpha=math.atan2(gry,grx)
        for0=K_rho*rho
        rotate=K_alpha*alpha - K_theta*gr0
        if rho!=0 and (alpha!=0 and gr0!=0):
            vel_msg.linear.x=for0
            vel_msg.angular.z=rotate
            return(vel_msg)
                
      
    
     
            
                
            
                
          
       
                
        
            
            
     
