#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs 
from std_msgs.msg import String
import numpy as np
from math import pi
#movegit has to be in the same folder as this script, otherwise specify location
from movegit import MoveGroupPythonIntefaceTutorial as move


if __name__=='__main__':
  try:
    #angle in degrees, there is a conversion in the function definition. 
    #the function is defined such that we only move the first joint, need to change this in the movegit script when ready
    move().go_to_joint_state(180) #the angle is related to the world frame. meaning: joint state of 180 will always be the same
    print move().robot.get_current_state().joint_state.position[0]
    print ("towards", "90") #remember this is hardcoded
  except rospy.ROSInterruptException:
    pass





