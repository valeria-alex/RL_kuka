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
wpose = geometry_msgs.msg.PoseStamped()
step_size=0.01
step_sizeZ=0.05
visited_states=[0,0,0,0,0]

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

def goal_update(state7):
    goal.append(state7)
    Q[state7,:]=10000
Q = np.zeros((18000,6))

epsilon = 1
alpha = 0.6 #0.9, 0.3
gamma = 0.9
count = 1
state_dict={}
move_dict={}  
action_dict={}
goal = range(0, 8620)

Q[0:8620,:]=10000  #praise Sofus
count = 0
with open(os.path.expanduser('~')+'/datafile1.txt', 'w') as the_file:
    the_file.write('episode, epsilon, start_time, finish_time,success, #steps,state \n')
num_dict={}
impossible_states=[]

def goal_update(state7):
    goal.append(state7)
    Q[state7,:]=10000

def checkKey(key, position1):
    global state_dict
    global count
    flagkey=False
    if key in state_dict:
        flagkey=True
    else:
        state_dict[key] = position1
        flagkey=False
    return flagkey
def checkKey_move(key, position2):
    
    flagkey=False
    if key in move_dict:
        flagkey=True
    else:
        move_dict.update({key:position2})
        flagkey=False
    
    return flagkey
def checkKey_visited(key, position2):
    
    flagkey=False
    
    if key in state_dict:
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
        flagkey=False
    
    return flagkey
def update(alpha, gamma, epsilon, state, next_state, selected,reward):
    Q_next = max(Q[next_state])
    Q[state][selected] = (1-alpha)*Q[state][selected]+ alpha*(reward + gamma*Q_next) #add more look aheads if dsadmnjksahdfjsa
    #print(Q[state][selected])

def select_action(state1):
    global move_dict
    state1=int(state1/18)*18
    if random.uniform(0, 1) < epsilon:
        select1 = random.choice(move_dict[state1])
        select = move_dict[state1].index(select1)
    else:
        listQ=[]
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
def force_predefine(state2,num1,num2,num3):
    
    state_dict[state2][0]=state_dict[state2][0]
    state_dict[state2][1]=state_dict[state2][1]
    state_dict[state2][2]=state_dict[state2][2]
    state_dict[state2][3]=num1
    state_dict[state2][4]=num2
    state_dict[state2][5]=num3
def force_pos_predefine(state8,pos8):
    for i in range(0,17):
            if i == 8:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,1,1,0)
            elif i == 1:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,1,1,1)
            elif i == 2:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,1,2,0)
            elif i == 3:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,1,2,1)
            elif i == 4:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,1,3,0)
            elif i == 5:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,1,3,1)
            elif i == 6:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,2,1,0)
            elif i == 7:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,2,1,1)
            elif i == 0:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,2,2,0)
            elif i == 9:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,2,2,1)
            elif i == 10:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,2,3,0)
            elif i == 11:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,2,3,1)
            elif i == 12:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,3,1,0)
            elif i == 13:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,3,1,1)
            elif i == 14:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,3,2,0)
            elif i == 15:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,3,2,1)
            elif i == 16:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,3,3,0)
            elif i == 17:
                checkKey(state8+i,list(pos8))
                force_predefine(state8+i,3,3,1)
def move_state(state1,pos):
    global state_dict
    global move_dict
    #print("state1 b4:" ,state1)
    
    state1=int(state1/18)
    #print("state1 after4:" ,state1)
    if state1 in action_dict:
        return action_dict[state1]
    else:
        
        checkKey_visited(state1,pos)
        pos_move=[]
        pos2=[1,1,2,2,2,0,0]
        #print("pos_move: ", state1)
        move_list=[0,1,2,3,4,5]
        available_states=[]
        force_pos_predefine(state1,pos)
        move_list[0] = (state1 - 100)
        move_list[1] = (state1 - 10)
        move_list[2] = (state1 - 1)
        move_list[3]= (state1 + 1)
        move_list[4] = (state1 + 10)
        move_list[5] = (state1 + 100)
        
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
              
            elif move_list[i] in impossible_states:
                pass
            elif move_list[i]>700:
                pass
            else:
                available_states.append(move_list[i])
                pos_move.append(e)
                e=e+1
        state1=state1*18
        checkKey_move(state1,pos_move)
        available_states = [x * 18 for x in available_states]
        for i in range(0,len(available_states)):
            if move_dict[state1][i]==0:
                pos2[0] = pos[0]
                pos2[1] = pos[1]
                pos2[2] = pos[2]-step_sizeZ
                checkKey(available_states[i],list(pos2))
                force_pos_predefine(available_states[i],pos2)
            elif move_dict[state1][i]==1:
                pos2[0] = pos[0]
                pos2[1] = pos[1]-step_size
                pos2[2] = pos[2]
                checkKey(available_states[i],list(pos2))
                force_pos_predefine(available_states[i],pos2)
            elif move_dict[state1][i]==2:
                pos2[0] = pos[0]-step_size
                pos2[1]= pos[1]
                pos2[2] = pos[2]
                
                checkKey(available_states[i],list(pos2))
                force_pos_predefine(available_states[i],pos2)
            
            elif move_dict[state1][i]==3:
                pos2[0] = pos[0]+step_size
                pos2[1] = pos[1]
                pos2[2] = pos[2]
                
                checkKey(available_states[i],list(pos2))
                force_pos_predefine(available_states[i],pos2)
            elif move_dict[state1][i]==4:
                pos2[0] = pos[0]
                pos2[1] = pos[1]+step_size
                pos2[2] = pos[2]
                
                checkKey(available_states[i],list(pos2))
                force_pos_predefine(available_states[i],pos2)
            elif move_dict[state1][i]==5:
                pos2[0] = pos[0]
                pos2[1] = pos[1]
                pos2[2]= pos[2]+step_sizeZ
                
                checkKey(available_states[i],list(pos2))
                force_pos_predefine(available_states[i],pos2)
        #print("move: ", move_dict)
        
    
        action_dict.update({state1:list(available_states)})
        #print("state: ", move_dict)
        return action_dict[state1]

def sense(st,pos8):
    #print("force: ", pos8)
    if pos8[3]==1 and pos8[4]==1 and pos8[5]==0:
        st=st+8
        print("1")
        num=0
    elif pos8[3]==1 and pos8[4]==1 and pos8[5]==1:
        st=st+1
        print("2")
        num=1
    elif pos8[3]==1 and pos8[4]==2 and pos8[5]==0:
        st=st+2
        print("3")
        num=2
    elif pos8[3]==1 and pos8[4]==2 and pos8[5]==1:
        st=st+3
        num=3
        print("4")
    elif pos8[3]==1 and pos8[4]==3 and pos8[5]==0:
        st=st+4
        num=4
        print("5")
    elif pos8[3]==1 and pos8[4]==3 and pos8[5]==1:
        st=st+5
        num=5
        print("6")
    elif pos8[3]==2 and pos8[4]==1 and pos8[5]==0:
        st=st+6
        num=6
        print("7")
    elif pos8[3]==2 and pos8[4]==1 and pos8[5]==1:
        st=st+7
        num=7
        print("8")
    elif pos8[3]==2 and pos8[4]==2 and pos8[5]==0:
        st=st+0
        num=8
        print("9")
    elif pos8[3]==2 and pos8[4]==2 and pos8[5]==1:
        st=st+9
        num=9
        print("10")
    elif pos8[3]==2 and pos8[4]==3 and pos8[5]==0:
        st=st+10
        num=10
        print("11")
    elif pos8[3]==2 and pos8[4]==3 and pos8[5]==1:
        st=st+11
        num=11
        print("12")
    elif pos8[3]==3 and pos8[4]==1 and pos8[5]==0:
        st=st+12
        num=12
        print("13")
    elif pos8[3]==3 and pos8[4]==1 and pos8[5]==1:
        st=st+13
        num=13
        print("14")
    elif pos8[3]==3 and pos8[4]==2 and pos8[5]==0:
        st=st+14
        num=14
        print("15")
    elif pos8[3]==3 and pos8[4]==2 and pos8[5]==1:
        st=st+15
        num=15
        print("16")
    elif pos8[3]==3 and pos8[4]==3 and pos8[5]==0:
        st=st+16
        num=16
        print("17")
    elif pos8[3]==3 and pos8[4]==3 and pos8[5]==1:
        st=st+17
        num=17
        print("18")
    return st,num
def discretizeZ(forcez):
    if forcez < 0:
        fz = 1#bad
    else:
        fz = 0#good
    return fz


def discretizexy(force): #input would be: fgina.wrench.force.x
    if force < -20: 
        f = 1 #range from -125 to 50  negative misalign
    elif force > -20  and force < 20:
        f = 2 #range from 0 to 50 good
    elif force > 20:
        f = 3 #range from 50 to 125 positive misalign
    return f

def move(pose):
    wpose.pose.orientation.w = 0.0
    wpose.pose.position.x = pose[0]+random.uniform(-0.0001,0.0001) 
    wpose.pose.position.y = pose[1]+random.uniform(-0.0001,0.0001) 
    wpose.pose.position.z = pose[2]+random.uniform(-0.0001,0.0001) 
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
        rew = -800
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
pos9=[0,1,2]
minPath=[]
all_state_visited=OrderedDict() 
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
    pos6=[1,2,3,2,2,0,0]
    pos5=[1,2,3,2,2,0,0]
    success = 0
    for i in range(1,1000):
        state=11790
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
        pos1[3]=discretizexy(fsub.wrench.force.x)
        pos1[4]=discretizexy(fsub.wrench.force.y)
        pos1[5]=discretizeZ(fsub.wrench.force.z)
        pos1[6]=1
        all_state_visited.clear()
        print(i)
        nosteps = 0
        actions = move_state(state,pos1)
        while True:
            start_time = time.time()
            pos2[0]=round(sub.pose.position.x,5)
            pos2[1]=round(sub.pose.position.y,5)
            pos2[2]=round(sub.pose.position.z,5)
            pos2[3]=discretizexy(fsub.wrench.force.x)
            pos2[4]=discretizexy(fsub.wrench.force.y)
            pos2[5]=discretizeZ(fsub.wrench.force.z)
            
            actions = move_state(state,pos2)
            
            print(state)
            stateF,num2=sense(state,pos2)
            checkKey(stateF,pos2)
            #print("actual state: ", stateF)
            
            selected = select_action(state)
            print("selected: ",selected)
            next_state = actions[selected]
            move(state_dict[next_state])
            nosteps += 1
            #print(state_dict)
            pos3[0]=state_dict[next_state][0]
            pos3[1]=state_dict[next_state][1]
            pos3[2]=state_dict[next_state][2]
            pos3[3]=discretizexy(fsub.wrench.force.x)
            pos3[4]=discretizexy(fsub.wrench.force.y)
            pos3[5]=discretizeZ(fsub.wrench.force.z)
            pos3[6]=1
            next_stateF,num1=sense(next_state,pos3)
            old_state=stateF
            checkKey(next_stateF,pos3)
            rate.sleep()
            time.sleep(0.2)
            if round(sub.pose.position.z,5) < 0.437 and abs(fsub.wrench.force.z)<150:
                reward = 100
                print("good state")
                #goal_update(next_stateF)
                update(alpha, gamma, epsilon, old_state, next_stateF, selected,reward)
                break
            elif num1!=8:
                reward=-1000
                update(alpha, gamma, epsilon, old_state, next_stateF, selected,reward)
            
            while num1!=8:
                
                actions=move_state(next_stateF,pos3)
                selected=select_action(int(next_stateF/18)*18)
                next_state1=actions[selected]
                move(state_dict[next_state1])
                nosteps += 1
                all_state_visited[state] = [sub.pose.position.x, sub.pose.position.y, sub.pose.position.z]
                time.sleep(0.2)
                if round(sub.pose.position.z,5) < 0.437 and abs(fsub.wrench.force.z)<150:
                    reward = 100
                    #goal_update(next_state1)
                    break
                else: 
                    reward = -1000
                
                
                update(alpha, gamma, epsilon, next_stateF,next_state1 ,selected,reward)
                pos3[0]=state_dict[next_state][0]
                pos3[1]=state_dict[next_state][0]
                pos3[2]=state_dict[next_state][0]
                pos3[3]=discretizexy(fsub.wrench.force.x)
                pos3[4]=discretizexy(fsub.wrench.force.y)
                pos3[5]=discretizeZ(fsub.wrench.force.z)
                pos3[6]=1
                old_state=next_stateF
                next_stateF,num1=sense(next_state1,pos6)
                
                rate.sleep()
                
            state=next_stateF
            if num1==8:
                #reward=0
                
                if round(sub.pose.position.z,5) < 0.437:
                    reward = 100
                    #goal_update(next_state3)
                    break
                else: 
                    reward = -1
                print(stateF)
                update(alpha, gamma, epsilon, old_state, next_stateF, selected,reward)
        finish_time = time.time()
        epsilon=epsilon*0.996     
        print("state break: ", state)
        move(state_dict[state])
         #was *0.8
        time.sleep(0.2)
        pos5[0]=round(sub.pose.position.x,5)
        pos5[1]=round(sub.pose.position.y,5)
        pos5[2]=round(sub.pose.position.z,5)
        pos5[3]=discretizexy(fsub.wrench.force.x)
        pos5[4]=discretizexy(fsub.wrench.force.y)
        pos5[5]=discretizeZ(fsub.wrench.force.z)
        state1,num2=sense(state,pos5)
        print("num goal: ", num2)
        print("x: ", pos5[0])
        print("y: ", pos5[1])
        print("z: ", pos5[2])
        #forcez=fsub.wrench.force.z
        print("forcez: ", fsub.wrench.force.z)
        if pos5[2] < 0.437 and abs(fsub.wrench.force.z)<150:
            reward=100
            update(alpha, gamma, epsilon, old_state, next_stateF, selected,reward)
            print("this is goal")
            success=success+1
        else:
            print("final fail: {} , {} , {} ".format(old_state,state ,selected))
            reward=-10000
            update(alpha, gamma, epsilon, old_state, next_stateF, selected,reward)
            
        with open(os.path.expanduser('~')+'/datafile1.txt', 'a') as the_file:
                the_file.write("{}, {},{},{},{},{},{} \n".format(i, epsilon, start_time, finish_time, success, nosteps,state))          
            
    np.savetxt("foo.csv", Q, delimiter=",")

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
    
    
