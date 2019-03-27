#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import random
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
    # move().go_to_joint_state(180) #the angle is related to the world frame. meaning: joint state of 180 will always be the same
    # print move().robot.get_current_state().joint_state.position[0]
    # print ("towards", "90") #remember this is hardcoded

    # ###FOR joint number 1: 
    move().go_to_joint_state(0, 90, 0, 0, 0, 0, 0)
    print move().robot.get_current_state().joint_state.position[1] * 180/pi
    # b = 0.0

    # while int(move().robot.get_current_state().joint_state.position[1]) not in range(-4, -2):
    #   # a = move().robot.get_current_state().joint_state.position[0]
    #   # if a == b:
    #   #   move().go_to_joint_state(0, 0, 0, 0)
    #   #   print ("I got stuck so I had to unstuck", a, b)


    #   move().go_to_joint_state(0, random.randint(-90,90), 0, 0)
    #   print ("arrived at", move().robot.get_current_state().joint_state.position[1])
    # #     b = move().robot.get_current_state().joint_state.position[1]

    # #FOR joint 0 - base link:
    # # move().go_to_joint_state(random.randint(-180,180), 0, 0, 0)
    # # print ("arrived at", move().robot.get_current_state().joint_state.position[0])
    # # #b = 0.0

    # # while int(move().robot.get_current_state().joint_state.position[0]) not in range(-3, -2):
    # #     move().go_to_joint_state(random.randint(-180.0,180.0), 0, 0, 0)
    # #     print ("arrived at", move().robot.get_current_state().joint_state.position[0])
    # #     if abs(move().robot.get_current_state().joint_state.position[0]) > 6.0:
    # #         move().robot.get_current_state().joint_state.position.reset()
    #     # else:
    #     #     posrad = move().robot.get_current_state().joint_state.position[0]

    # #   b = move().robot.get_current_state().joint_state.position[0]





  except rospy.ROSInterruptException:
    pass





