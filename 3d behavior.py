#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import random
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs 
from std_msgs.msg import String, Float64
import numpy as np
from math import pi
#movegit has to be in the same folder as this script, otherwise specify location
from movegit import MoveGroupPythonIntefaceTutorial as move
#from gazebo_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped, WrenchStamped
import math
from moveit_commander.conversions import pose_to_list
from collections import OrderedDict 
import os
from numpy import genfromtxt
wpose = geometry_msgs.msg.PoseStamped()
step_size=0.01
step_sizeZ=0.05
visited_states=[0,0,0,0,0]
Q = genfromtxt(os.path.expanduser('~')+'/foo.csv', delimiter=',')
with open(os.path.expanduser('~')+'/datafile13.txt', 'w') as the_file:
    the_file.write('episode, epsilon, start_time, finish_time,success, #steps,state \n')
def cb(data):
    global sub
    sub = data
def fcb(data):
    global fsub
    fsub = data
def open_g():
    pub1 = rospy.Publisher('/bh/finger1_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/bh/finger2_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/bh/finger3_controller/command', Float64, queue_size=10)
    pub1.publish(0.0)
    pub2.publish(0.0)
    pub3.publish(0.0)


def close_g():
    pub1 = rospy.Publisher('/bh/finger1_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/bh/finger2_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/bh/finger3_controller/command', Float64, queue_size=10)
    pub1.publish(-0.5)
    pub2.publish(-0.5)
    pub3.publish(-0.5)


#Q = np.zeros((1000,6))
#Q[600:]=-1000000000000

epsilon = 0
alpha = 0.6 #0.9, 0.3
gamma = 0.9
count = 1

state_dict={}
move_dict={}  
action_dict={}
goal = range(0, 320)

#Q[0:320,:]=10000  #praise Sofus
count = 0

num_dict={}
impossible_states=[]
    
def goal_update(state7):
    goal.append(state7)
    Q[state7,:]=10000
def checkKey(key, position1):
    global state_dict
    global count
    #count = count +1
    flagkey=False
    #if count < 4:
        #print("state_dict before ", state_dict)
    if key in state_dict:
        #print("key found {} = {}".format(key,state_dict[key]))
        flagkey=True
    else:
        #state_dict.update({key:position1})
        state_dict[key] = position1
        #move_state(selected)
        #print("new key: {} = {}".format(key,state_dict[key]))
        flagkey=False
    #if count < 4:
        #print("state_dict after ", state_dict)
    return flagkey
def checkKey_move(key, position2):
    
    flagkey=False
    
    if key in move_dict:
        #print("key found {} = {}".format(key,state_dict[key]))
        flagkey=True
    else:
        move_dict.update({key:position2})
        #move_dict[key] = position2
        flagkey=False
    
    return flagkey
def checkKey_visited(key, position2):
    
    flagkey=False
    
    if key in state_dict:
        #print("key found {} = {}".format(key,state_dict[key]))
        if state_dict[key][6]==1:
            pass
        else:
            state_dict[key][3]=position2[3]
            state_dict[key][4]=position2[4]
            state_dict[key][5]=position2[5]
            state_dict[key][6]=1
        flagkey=True
        
    else:
        state_dict.update({key:position2})
        #move_dict[key] = position2
        flagkey=False
    
    return flagkey


def select_action(state1):
    global move_dict
    if random.uniform(0, 1) < epsilon:
        select1 = random.choice(move_dict[state1])
        #print("move_dict: ", move_dict[state1])
        #print("move_dict2: ", move_dict)
        #print("select1: ", select1)
        select = move_dict[state1].index(select1)
    else:
        listQ=[]
        #print("move_dict1: ", move_dict[state1])
        for i in move_dict[state1]:
            listQ.append(Q[state1,i])
        #print("listQ: ",listQ)
        if max(listQ) == 0:
            select1 = random.choice(move_dict[state1])
            select = move_dict[state1].index(select1)
            #print("select: ",select)
        else:
            select = np.where(Q[state1]==max(listQ))[0][0]
    return(select)

def move_state(state1,pos):
    global state_dict
    global move_dict
    if state1 in action_dict:
        return action_dict[state1]
    else:

        #checkKey(2, [11, 12, 13])
        pos_move=[]
        pos2=[1,1,2]
        #print("pos_move: ", state1)
        move_list=[0,1,2,3,4,5]
        available_states=[]
        
        #
        pos2[0] = pos[0]
        pos2[1] = pos[1]
        pos2[2] = pos[2]-step_sizeZ
        move_list[0] = state1 - 100
        checkKey(move_list[0],list(pos2))
        #
        #
        pos2[0] = pos[0]
        pos2[1] = pos[1]-step_size
        pos2[2] = pos[2]
        move_list[1] = state1 - 10
        checkKey(move_list[1],list(pos2))
        #
        #
        pos2[0] = pos[0]-step_size
        pos2[1]= pos[1]
        pos2[2] = pos[2]
        move_list[2] = state1 - 1
        checkKey(move_list[2],list(pos2))
        #
        pos2[0] = pos[0]+step_size
        pos2[1] = pos[1]
        pos2[2] = pos[2]
        move_list[3]= state1 + 1
        checkKey(move_list[3],list(pos2))
        #
        #
        pos2[0] = pos[0]
        pos2[1] = pos[1]+step_size
        pos2[2] = pos[2]
        move_list[4] = state1 + 10
        checkKey(move_list[4],list(pos2))
        #
        #
        pos2[0] = pos[0]
        pos2[1] = pos[1]
        pos2[2]= pos[2]+step_sizeZ
        move_list[5] = state1 + 100
        checkKey(move_list[5],list(pos2))
        #

        #
        e=0
        for i in range(0,6):
            if state1 < 0 or move_list[i] < 0 or state1 > 999 or move_list[i] > 999:
                pass
            elif state1 % 10 == 0 and (move_list[i] - 9) % 10 == 0 or (state1 - 9) % 10 == 0 and move_list[i] % 10 == 0:
                pass
            elif int((state1 % 100)/10) == 0 and int((move_list[i] % 100)/10) == 9:
                pass
            elif int((move_list[i] % 100)/10) == 0 and int((state1 % 100)/10) == 9:
                pass
            
            elif move_list[i]>700:
                pass
            else:
                available_states.append(move_list[i])
                pos_move.append(e)
                e=e+1
        checkKey_move(state1,pos_move)
        #print("move: ", move_dict)
        action_dict.update({state1:list(available_states)})
        #print(action_dict)
        #for key in state_dict:
            #if state_dict[key][6]==1:
                #print("visited states {} = {}".format(key,state_dict[key]))
        return action_dict[state1]

def discretizeZ(forcez):
    if forcez < 0:
        fz = 1#bad
    else:
        fz = 0#good
    return fz


def discretizexy(force): #input would be: fgina.wrench.force.x
    if force < -50: 
        f = 1 #range from -125 to 50  negative misalign
    elif force > -50  and force < 50:
        f = 2 #range from 0 to 50 good
    elif force > 50:
        f = 3 #range from 50 to 125 positive misalign
    return f

def move(pose):
    wpose.pose.orientation.w = 0.0
    wpose.pose.position.x = pose[0]
    wpose.pose.position.y = pose[1]
    wpose.pose.position.z = pose[2] 
    wpose.pose.orientation.x = 0.0
    wpose.pose.orientation.y = 1.0
    wpose.pose.orientation.z = 0.0
    pub.publish(wpose)
    close_g()
    #print ('here', sub.pose.position.x, sub.pose.position.y, sub.pose.position.z)
    time.sleep(0.1)

def check(state,norm_force,next_state1):
    flagbreak = False
    global count
    global state_dict
    if norm_force>50:
        #backup()
        print (visited_states)     
        move(state_dict[visited_states[count]])
   
        next_state2=visited_states[count]
        rew = -1000000000
        print("experience, back..")
        count = count +1

        if count >= 4:
            flagbreak = True
            count = 0
        if visited_states[0]==visited_states[1]:
            flagbreak = True
            count=0
    else:
        visited_states.pop(4)
        visited_states.insert(0,state)
        next_state2=next_state1
        rew = 0
        count=0
    return next_state2,rew,flagbreak

def check_collision(state,next_state,fx,fy,fz):
    #global count
    #flagB=False
    if fx ==1:
        print("negative misaligned in x")
    elif fy==1:
        print("negative misaligned in y")
    elif fx==3:
        print("positive misaligned in x")
    elif fy==3:
        print("positive misaligned in y")
    elif fz==1:
        print("high force in z")
    #else: 
        #visited_states.pop(4)
        #visited_states.insert(0,state)
        #reward2=0
        #next_state4=next_state
        #count=0
    #return reward2,next_state4,flagB

state455={}
state355={}
state255={}
state155={}
count455=0
count355=0
count255=0
count155=0
pos9=[0,1,2]
minPath=[]
all_state_visited=OrderedDict() 
success = 0
len_visited=[]
if __name__ == '__main__':
    
    pub = rospy.Publisher('/bh/command/CartesianPose', PoseStamped, queue_size=10)
    m1 = rospy.Subscriber('/bh/state/CartesianPose', PoseStamped, cb, queue_size=10)
    m2 = rospy.Subscriber('/bh/state/CartesianWrench', WrenchStamped, fcb, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    sub = PoseStamped()
    fsub = WrenchStamped()
    time.sleep(0.5) #it takes a bit for my PC to initialize all the ROS stufff, you might not need this (its just a delay)
    pos1=[1,2,3,2,2,0,0]
    pos2=[1,2,3,2,2,0,0]
    pos3=[1,2,3,2,2,0,0]
    pos4=[1,2,3,2,2,0,0]
    
    for i in range(1,10):
        state=655
        wpose.pose.orientation.w = 0.0
        wpose.pose.position.x = 0.391
        wpose.pose.position.y = -0.242
        wpose.pose.position.z = 0.470
        wpose.pose.orientation.x = 0.0
        wpose.pose.orientation.y = 1.0
        wpose.pose.orientation.z = 0.0
        pub.publish(wpose)
        close_g()
        time.sleep(2) 
        #print('there', round(sub.pose.position.x,2), round(sub.pose.position.y,2), round(sub.pose.position.z,2))
        pos1[0]=round(sub.pose.position.x,5)
        pos1[1]=round(sub.pose.position.y,5)
        pos1[2]=round(sub.pose.position.z,5)

        all_state_visited.clear()
        print(i)
        nosteps = 0
        while True:
            start_time = time.time()
            pos2[0]=round(sub.pose.position.x,5)
            pos2[1]=round(sub.pose.position.y,5)
            pos2[2]=round(sub.pose.position.z,5)

            actions = move_state(state,pos2)
            print(state)
     
            selected = select_action(state)
            #print("selected: ",selected)
            next_state = actions[selected]
            move(state_dict[next_state])
            old_state=state
            
            pos3[0]=round(sub.pose.position.x,5)
            pos3[1]=round(sub.pose.position.y,5)
            pos3[2]=round(sub.pose.position.z,5)
            
            norm_force = math.sqrt(fsub.wrench.force.x*fsub.wrench.force.x+ fsub.wrench.force.y*fsub.wrench.force.y + fsub.wrench.force.z*fsub.wrench.force.z)
            #print("norm: ",norm_force)
            next_state3,reward1,flagb = check(state,norm_force,next_state)
        
            all_state_visited[state] = [sub.pose.position.x, sub.pose.position.y, sub.pose.position.z,norm_force]
            
            if round(sub.pose.position.z,5) < 0.433 and norm_force<300:
                reward = 100
                goal_update(next_state3)
                break
            else: 
                reward = -1
            reward = reward1 + reward
            #update(alpha, gamma, epsilon, state, next_state, selected,reward)
            old_state=state
            state=next_state3
            nosteps += 1
            
            #print("state after experience force: ", state)
            if flagb:
                print("breaking..")
                break
            else:
                pass
            #print("z: ", round(sub.pose.position.z,5))
            time.sleep(0.5)
            rate.sleep()
        finish_time = time.time()
             
        epsilon = epsilon*0.996
        #move(state_dict[143])
        print("state break: ", state)

        
        move(state_dict[state])
         #was *0.8
        time.sleep(0.2)
        if round(sub.pose.position.z,5) < 0.433:
            
            
           
            #actions = move_state(old_state,pos9)
            
           
            print("this is goal")
            success=success+1
        with open(os.path.expanduser('~')+'/datafile13.txt', 'a') as the_file:
                the_file.write("{}, {},{},{},{},{},{} \n".format(i, epsilon, start_time, finish_time, success, nosteps,state))   
            
    #np.savetxt("foo.csv", Q, delimiter=",")

    print("done")
    
    
    print("testing: ")
    wpose.pose.orientation.w = 0.0
    wpose.pose.position.x = 0.5
    wpose.pose.position.y = -0.4
    wpose.pose.position.z = 0.38
    wpose.pose.orientation.x = 0.0
    wpose.pose.orientation.y = 1.0
    wpose.pose.orientation.z = 0.0
    pub.publish(wpose)
    close_g()
    time.sleep(2)
    
    
