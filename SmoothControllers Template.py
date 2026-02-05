#!/usr/bin/env python  
 
from geometry_msgs.msg import Twist
import math
 
class SmoothController1:
    def __init__(self, goalIx, goalIy):
        """
        Initialize the class with the position of the goal in the robot ref.
        frame (goalIx, goalIy).
        """
        ???
 
    def get_twist(self, x, y, theta):
        """
        Given the current pose of the robot (x, y, theta) compute and return
        the appropriate Twist message.
        """
        ???