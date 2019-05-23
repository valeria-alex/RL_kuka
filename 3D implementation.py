#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
import geometry_msgs.msg
import pandas as pd
import numpy as np
import random 
import time
import math

Q = np.zeros((1000, 6))

epsilon = 1
alpha = 0.6
count = 1
success_episodes = 0
depth = 8

def cb(data):
    global gina
    gina = data

def fcb(data):
    global fgina
    fgina = data

if __name__ == '__main__':

    pub = rospy.Publisher('/bh/command/CartesianPose', PoseStamped, queue_size=1)
    m1 = rospy.Subscriber('/bh/state/CartesianPose', PoseStamped, cb, queue_size=10)
    f1 = rospy.Subscriber('/bh/state/CartesianWrench', WrenchStamped, fcb, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    gina = PoseStamped()
    fgina = WrenchStamped()
    time.sleep(0.1)
    x_coor = gina.pose.position.x 
    y_coor = gina.pose.position.y 
    z_coor = gina.pose.position.z


def available_actions(state):
    z_neg = state - 100
    y_neg = state - 10
    x_neg = state - 1
    
    z_pos = state + 1
    y_pos = state + 10
    x_pos = state + 100


    return(z_neg, y_neg, x_neg, z_pos, y_pos, x_pos)

def select_action():
    if random.uniform(0, 1) < epsilon:
        select = random.randint(0,5)
    elif max(abs(Q[state])) == 0:
        select = random.randint(0,5)
    else:
        select = np.argmax(Q[state])
    return(select)

#def update(alpha, epsilon, state, next_state, selected, reward):
#def update(alpha, epsilon, state, next_state, next2_state, selected, reward):
#def update(alpha, epsilon, state, next_state, next2_state, next3_state, selected, reward):
def update(alpha, epsilon, state, next_state, next2_state, next3_state, next4_state, selected, reward):
#def update(alpha, epsilon, state, next_state, next2_state, next3_state, next4_state, next5_state, selected, reward):
#def update(alpha, epsilon, state, next_state, next2_state, next3_state, next4_state, next5_state, next6_state, selected, reward):
    Q_next = max(Q[next_state])
    Q_next2 = max(Q[next2_state])
    Q_next3 = max(Q[next3_state])
    Q_next4 = max(Q[next4_state])
    #Q_next5 = max(Q[next5_state])
    #Q_next6 = max(Q[next6_state])
    #Q[state][selected] = (1-alpha)*Q[state][selected]+ alpha*(reward + (0.5**1)*Q_next)
    #Q[state][selected] = (1-alpha)*Q[state][selected]+ alpha*(reward + (0.5**1)*Q_next + (0.5**2)*Q_next2)
    #Q[state][selected] = (1-alpha)*Q[state][selected]+ alpha*(reward + (0.5**1)*Q_next + (0.5**2)*Q_next2 + (0.5**3)*Q_next3)
    Q[state][selected] = (1-alpha)*Q[state][selected]+ alpha*(reward + (0.5**1)*Q_next + (0.5**2)*Q_next2 + (0.5**3)*Q_next3 + (0.5**4)*Q_next4)
    #Q[state][selected] = (1-alpha)*Q[state][selected]+ alpha*(reward + (0.5**1)*Q_next + (0.5**2)*Q_next2 + (0.5**3)*Q_next3 + (0.5**4)*Q_next4 + (0.5**5)*Q_next5)
    #Q[state][selected] = (1-alpha)*Q[state][selected]+ alpha*(reward + (0.5**1)*Q_next + (0.5**2)*Q_next2 + (0.5**3)*Q_next3 + (0.5**4)*Q_next4 + (0.5**5)*Q_next5 + (0.5**6)*Q_next6)


#----------TRAINING------------------------

force_measurement = [1]
indicator = 0

with open('alpha06v1.txt', 'w') as the_file:
    the_file.write('episode, epsilon, start_time, finish_time, 1/0--success, #steps, \n')

for i in range(1,800):
    del force_measurement[:]  
    state = 455
    print("Initial position")
    initialx = 0.391
    initialy = -0.242
    initialz = 0.495
    wpose = geometry_msgs.msg.PoseStamped()
    wpose.pose.position.x = initialx
    wpose.pose.position.y = initialy
    wpose.pose.position.z = initialz
    wpose.pose.orientation.w = 0.0
    wpose.pose.orientation.x = 0.0
    wpose.pose.orientation.y = 1.0
    wpose.pose.orientation.z = 0.0
    pub.publish(wpose)

    print(initialx)
    print(initialy)
    print(initialz)
    time.sleep(5)

    x_coor = gina.pose.position.x 
    y_coor = gina.pose.position.y 
    z_coor = gina.pose.position.z

    start_time = time.time()
    nosteps = 0
    while int(state/100) != depth:
        #1 LOOK AHEAD
        actions = available_actions(state)
        selected = select_action()
        norm_force = math.sqrt(fgina.wrench.force.x*fgina.wrench.force.x+ fgina.wrench.force.y*fgina.wrench.force.y + fgina.wrench.force.z*fgina.wrench.force.z)
        next_state = actions[selected]

        if state < 300 or next_state < 300 or state > 999 or next_state > 999:
            next_state = state
        elif state % 10 == 0 and (next_state - 9) % 10 == 0 or (state - 9) % 10 == 0 and next_state % 10 == 0:
            next_state = state
        elif int((state % 100)/10) == 0 and int((next_state % 100)/10) == 9:
            next_state = state
        elif int((next_state % 100)/10) == 0 and int((state % 100)/10) == 9:
            next_state = state
        else:
            pass

        #2 LOOK AHEAD
        actions2 = available_actions(next_state)
        selected2 = select_action()
        next2_state = actions[selected]

        if next_state < 300 or next2_state < 300 or next_state > 900 or next2_state > 900:
            next2_state = next_state
        elif next_state % 10 == 0 and (next2_state - 9) % 10 == 0 or (next_state - 9) % 10 == 0 and next2_state % 10 == 0:
            next2_state = next_state
        elif int((next_state % 100)/10) == 0 and int((next2_state % 100)/10) == 9:
            next2_state = next_state
        elif int((next2_state % 100)/10) == 0 and int((next_state % 100)/10) == 9:
            next2_state = next_state
        else:
            pass

        #3 LOOK AHEAD
        actions3 = available_actions(next_state)
        selected3 = select_action()
        next3_state = actions[selected]

        if next2_state < 300 or next3_state < 300 or next2_state > 900 or next3_state > 900:
            next3_state = next2_state
        elif next2_state % 10 == 0 and (next3_state - 9) % 10 == 0 or (next2_state - 9) % 10 == 0 and next3_state % 10 == 0:
            next3_state = next2_state
        elif int((next2_state % 100)/10) == 0 and int((next3_state % 100)/10) == 9:
            next3_state = next2_state
        elif int((next3_state % 100)/10) == 0 and int((next2_state % 100)/10) == 9:
            next3_state = next2_state
        else:
            pass

        #4 LOOK AHEAD
        actions4 = available_actions(next_state)
        selected4 = select_action()
        next4_state = actions[selected]

        if next3_state < 300 or next4_state < 300 or next3_state > 900 or next4_state > 900:
            next4_state = next3_state
        elif next3_state % 10 == 0 and (next4_state - 9) % 10 == 0 or (next3_state - 9) % 10 == 0 and next4_state % 10 == 0:
            next4_state = next3_state
        elif int((next3_state % 100)/10) == 0 and int((next4_state % 100)/10) == 9:
            next4_state = next3_state
        elif int((next4_state % 100)/10) == 0 and int((next3_state % 100)/10) == 9:
            next4_state = next3_state
        else:
            pass

        #5 LOOK AHEAD
        #actions5 = available_actions(next_state)
        #selected5 = select_action()
        #next5_state = actions[selected]

        #if next4_state < 300 or next5_state < 300 or next4_state > 900 or next5_state > 900:
        #    next5_state = next4_state
        #elif next4_state % 10 == 0 and (next5_state - 9) % 10 == 0 or (next4_state - 9) % 10 == 0 and next5_state % 10 == 0:
        #    next5_state = next4_state
        #elif int((next4_state % 100)/10) == 0 and int((next5_state % 100)/10) == 9:
        #    next5_state = next4_state
        #elif int((next5_state % 100)/10) == 0 and int((next4_state % 100)/10) == 9:
        #    next5_state = next4_state
        #else:
        #    pass

        #6 LOOK AHEAD
        #actions6 = available_actions(next_state)
        #selected6 = select_action()
        #next6_state = actions[selected]

        #if next5_state < 300 or next6_state < 300 or next5_state > 900 or next6_state > 900:
        #    next6_state = next5_state
        #elif next5_state % 10 == 0 and (next6_state - 9) % 10 == 0 or (next5_state - 9) % 10 == 0 and next6_state % 10 == 0:
        #    next6_state = next5_state
        #elif int((next5_state % 100)/10) == 0 and int((next6_state % 100)/10) == 9:
        #    next6_state = next5_state
        #elif int((next6_state % 100)/10) == 0 and int((next5_state % 100)/10) == 9:
        #    next6_state = next5_state
        #else:
        #    pass
        
        print("Episode: ", count,"State: ", state)

        x = 0.2*-1*float((next_state % 10) - (state % 10))/100
        y = 0.2*-1*float(int((next_state % 100)/10) - int((state % 100)/10))/100
        z = 0.8*-1*float(int(next_state / 100) - int(state / 100))/100

        x_coor += x
        y_coor += y
        z_coor += z

        wpose = geometry_msgs.msg.PoseStamped()
        wpose.pose.position.x = x_coor
        wpose.pose.position.y = y_coor
        wpose.pose.position.z = z_coor
        wpose.pose.orientation.w = 0.0
        wpose.pose.orientation.x = 0.0
        wpose.pose.orientation.y = 1.0
        wpose.pose.orientation.z = 0.0
        pub.publish(wpose)
        time.sleep(0.2)


            #force_measurement.append(1)
            #print(force_measurement)
            #if len(force_measurement) == 6:
            #    print("Breaking - Too many threshold exceedings!")
            #    break
            #else:
            #    pass
            #wpose = geometry_msgs.msg.PoseStamped()
            #wpose.pose.position.x = x_coor 
            #wpose.pose.position.y = y_coor 
            #wpose.pose.position.z = 0.5
            #wpose.pose.orientation.w = 0.0
            #wpose.pose.orientation.x = 0.0
            #wpose.pose.orientation.y = 1.0
            #wpose.pose.orientation.z = 0.0
            #pub.publish(wpose)
            #time.sleep(1)
            #print("this is state: ",state)
            #print("this is next_state: ", next_state)
            #next_state = (state%100)+100
            #print("new next state: ", next_state)



        if int(next_state/100) == depth:
            reward = 1000
        elif norm_force >= 125:
            reward = -1*norm_force
        else:
            reward = -0.5*abs(depth-int(state/100)) + -0.8*abs(5-int((state % 100)/10))+ -0.8*abs((5-(state % 10)))
        
        print("Reward: ", reward, "Force: ", norm_force)
        print("Epsilon: ",float(epsilon))
        print("Success of episodes: ", success_episodes)

        #update(alpha, epsilon, state, next_state, selected, reward)
        #update(alpha, epsilon, state, next_state, next2_state, selected, reward)
        #update(alpha, epsilon, state, next_state, next2_state, next3_state, selected, reward)
        update(alpha, epsilon, state, next_state, next2_state, next3_state, next4_state, selected, reward)
        #update(alpha, epsilon, state, next_state, next2_state, next3_state, next4_state, next5_state, selected, reward)
        #update(alpha, epsilon, state, next_state, next2_state, next3_state, next4_state, next5_state, next6_state, selected, reward)


        if norm_force > 120:
            print("Force interference!")
            break
        #else:
        #    state = next_state

        state = next_state
        nosteps += 1

    epsilon = epsilon*0.996
    success = 0
    if int(state/100) == depth:
        print("Goal reached, going to initial position")
        success_episodes += 1
        success = 1
    else:
        print("Failed!")


    count += 1
    finish_time = time.time()
    with open('alpha06v1.txt', 'a') as the_file:
        the_file.write("{}, {},{},{},{},{}, \n".format(i, epsilon, start_time, finish_time, success, nosteps))

    pd.DataFrame(Q).to_csv("/home/fjojensen/catkin_ws/src/hello_ros/scripts/3D_final.csv")

    if success == 1:
        indicator += 1
    else: 
        indicator = 0
    if indicator > 15:
        #print ("more than 10 iterations with less than 10 steps to the goal")
        pass
   


#---------PRINTING-------------------------

#CHANGE THE LOCATION OF THE .CSV FILE
#IF YOUR CYLINDER IS NOT 200MM IN LENGTH, YOU NEED TO ADJUST THE HEIGHT (MAKE IT START CLOSER TO THE ENGINE BLOCK)