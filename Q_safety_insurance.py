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

Q = np.zeros((1000000, 6)) #makemanually

epsilon = 1
alpha = 0.6
count = 1
success_episodes = 0
depth = 7
high = 1

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
    
    x_pos = state + 1
    y_pos = state + 10
    z_pos = state + 100

    return(z_neg, y_neg, x_neg, x_pos, y_pos, z_pos)
    #return(move(0, 0, -0.4), move(0, -0.2, 0), move(-0.2, 0, 0), move(0, 0, 0.4), move(0, 0.2, 0), move(0.2, 0, 0))

def select_action():
    if random.uniform(0, 1) < epsilon:
        select = random.randint(0,5)
        action_type = 9
    elif max(abs(Q[statewithforce])) == 0: 
        select = random.randint(0,5)
        action_type = 9
    else:
        select = np.argmax(Q[statewithforce])
        action_type = 1 #it never gets
    return(select, action_type)

def move(x, y, z):
    wpose.pose.position.x = x # +random.uniform(-0.005,0.005) 
    wpose.pose.position.y = y # +random.uniform(-0.005,0.005) 
    wpose.pose.position.z = z #+random.uniform(-0.005,0.005) 
    wpose.pose.orientation.x = 0.0
    wpose.pose.orientation.y = 1.0
    wpose.pose.orientation.z = 0.0
    wpose.pose.orientation.w = 0.0
    pub.publish(wpose)
    time.sleep(0.4)

def getstate(state): #idea 3: eliminate x and y from state
    # x = int(gina.pose.position.x*10)
    # y = int(gina.pose.position.y*10)
    # z = int(gina.pose.position.z*10) 
    fx = discreteCC(fgina.wrench.force.x)
    fy = discreteCC(fgina.wrench.force.y)
    fz = 1
    if fgina.wrench.force.z < 0:
        fz = 0
    statefinal = state*1000 + fx*100 + fy*10 + fz
    return statefinal


def discreteCC(force): #only x and y
    value = 2
    if force < -10:
        value = 1
    elif force > 10:
        value = 3
    return value


def rewardfunc(z, fx, fy, fz, n_f, felt):
    bonus = 120
    global high
    if felt and n_f < 20:
        bonus = 100
        #high = 10
    elif felt and n_f > 20: #high
        high *= 10
        bonus = -10*high
    else:
        high = 1
    print ("high", high)
    #distance to goal??
    r = 50*z - 4*abs(fx) - 4*abs(fy) - 5*abs(fz)
    #r = 210*z - 40*abs(fx) - 40*abs(fy) - 50*abs(fz) #+ bonus# - 10*nosteps/epsilon #can there be const*z of next_state? / ?
    return r

def forcerewardthing(fx):
    if fx == 2:
        varx = 600
    elif fx == 1:
        varx = -300
    else:
        varx = -100
    return varx

def reward_discrete(z, fx, fy, fz):
    if fz == 0: 
        varZ = - 300
    else:
        varZ = 300
    r = 100*z + forcerewardthing(fx)*(fx) + forcerewardthing(fy)*(fy) + varZ #1
    return r


def update(alpha, epsilon, ew, next_state, selected, reward):
#def update(alpha, epsilon, state, next_state, next2_state, selected, reward):
    Q_next = max(Q[next_state]) 
    #Q_next2 = max(Q[next2_state])
    Q[ew][selected] = round((1-alpha)*Q[ew][selected]+ alpha*(reward + 0.9*Q_next), 2)
    #Q[state][selected] = (1-alpha)*Q[state][selected]+ alpha*(reward + 0.6*Q_next + 0.2*Q_next2)


#----------TRAINING------------------------
ranger = 2000
force_measurement = [1]
indicator = 0
beta = 0
with open('datafile-10ftrew122.txt', 'w') as the_file:
    the_file.write('episode, epsilon, start_time, finish_time, 1/0--success, #steps, lars, statewithforce, initialx, initialy,  \n')

with open('datafile-actions122.txt', 'w') as the_file:
    the_file.write('episode, epsilon, #steps, lars, statewithforce, selected, \n')

for i in range(1, ranger): #weprollyneedtoexploremoar
    del force_measurement[:]  
    state = 355
    print("Initial position")
    initialx = 0.391 +0.01*random.uniform(-1, 1)
    initialy = -0.242 +0.01*random.uniform(-1, 1)
    initialz = 0.502
    wpose = geometry_msgs.msg.PoseStamped()#disc ini
    wpose.pose.position.x = initialx #random.uniform(-1,1) #Made this larger on negative  Y because in my sim it wasnt exploring
    wpose.pose.position.y = initialy #random.uniform(-1,1) #Put this back "low" when object = "optimal"
    wpose.pose.position.z = initialz #+0.0001*random.uniform(-0.5,0.55) 
    wpose.pose.orientation.w = 0.0
    wpose.pose.orientation.x = 0.0
    wpose.pose.orientation.y = 1.0
    wpose.pose.orientation.z = 0.0
    pub.publish(wpose)

    print(initialx)
    print(initialy)
    print(initialz)
    time.sleep(2)

    x_coor = gina.pose.position.x 
    y_coor = gina.pose.position.y 
    z_coor = gina.pose.position.z

    start_time = time.time()
    nosteps = 0
    statewithforce = getstate(state)
    while int(state/100) != depth: #z + exceeding #steps (based on epsilon)
        #1 LOOK AHEAD
        statewithforce = getstate(state)
        actions = available_actions(state)
        selected, lars = select_action()
        norm_force = math.sqrt(fgina.wrench.force.x*fgina.wrench.force.x+ fgina.wrench.force.y*fgina.wrench.force.y + fgina.wrench.force.z*fgina.wrench.force.z)
        next_state = actions[selected]
        if norm_force > 20: #for state, the one we update. 
            felt_force = True 
        else: 
            felt_force = False

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
        #actions2 = available_actions(next_state)
        #selected2 = select_action()
        #next2_state = actions[selected]

        #if next_state < 300 or next2_state < 300 or next_state > 900 or next2_state > 900:
        #    next2_state = next_state
        #elif next_state % 10 == 0 and (next2_state - 9) % 10 == 0 or (next_state - 9) % 10 == 0 and next2_state % 10 == 0:
        #    next2_state = next_state
        #elif int((next_state % 100)/10) == 0 and int((next2_state % 100)/10) == 9:
        #    next2_state = next_state
        #elif int((next2_state % 100)/10) == 0 and int((next_state % 100)/10) == 9:
        #    next2_state = next_state
        #else:
        #    pass
        #
        print("Episode: ", count,"State: ", state, statewithforce, "Next state: ", next_state, lars)
        #int(next_state/1000)
        #int(state/1000)



        ############CASPER made THIS:::::::::
        # posvar1=0.0001*random.uniform(1,1)
        # posvar2=0.0001*random.uniform(1,1)
        # posvar3=0.0001*random.uniform(1,1)

        # x = (0.2+(posvar1-posvar1_old))*-1*float((next_state % 10) - (state % 10))/100
        # y = (0.2+(posvar1-posvar2_old))*-1*float(int((next_state % 100)/10) - int((state % 100)/10))/100
        # z = (0.8+(posvar3-posvar3_old))*-1*float(int(next_state / 100) - int(state / 100))/100

        # posvar1_old=posvar1
        # posvar2_old=posvar2
        # posvar3_old=posvar3


        with open('datafile-actions122.txt', 'a') as the_file:
            the_file.write("{},{},{},{},{},{}, \n".format(i, epsilon, nosteps + 1, lars, statewithforce, selected))




        x = 0.2*-1*float((next_state % 10) - (state % 10))/100
        y = 0.2*-1*float(int((next_state % 100)/10) - int((state % 100)/10))/100
        z = 0.8*-1*float(int(next_state / 100) - int(state / 100))/100

        x_coor += x
        y_coor += y
        z_coor += z
        wpose = geometry_msgs.msg.PoseStamped()
        wpose.pose.position.x = x_coor #add uncertainity 
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
        norm_force = math.sqrt(fgina.wrench.force.x*fgina.wrench.force.x+ fgina.wrench.force.y*fgina.wrench.force.y + fgina.wrench.force.z*fgina.wrench.force.z)
        
        reward = rewardfunc(int(statewithforce/100000), fgina.wrench.force.x, fgina.wrench.force.y, fgina.wrench.force.z, norm_force, felt_force)

        if int(next_state/100) == depth and norm_force < 20: #20 
            reward += 100000 #- norm_force * z ?
        # elif norm_force >= 40:
        #     reward = -1.3*norm_force
        #     #for th second test
        # #     beta = 1
        # # elif beta == 1:
        # #     reward == -0.5*abs(depth-int(state/100)) + -0.8*abs(5-int((state % 100)/10))+ -0.8*abs((5-(state % 10))) + 200
        # #     beta = 0
        # else:
        #     #reward = - norm_force + 10*z
        #     #reward = 10*z - forces
        #     reward = -0.5*abs(depth-int(state/100)) + -0.8*abs(5-int((state % 100)/10))+ -0.8*abs((5-(state % 10)))
        
        print("Reward: ", reward, "Force: ", norm_force)
        print("Epsilon: ",float(epsilon))
        print("forces",fgina.wrench.force.x, fgina.wrench.force.y, fgina.wrench.force.z)
        print("Success of episodes: ", success_episodes)
        print("                                 ", "###", count, "###", "                                 " )

        next_state = getstate(next_state)
        #statewithforce = getstate(state)
        update(alpha, epsilon, statewithforce, next_state, selected, reward)
        #print (Q[statewithforce])
        
        #update(alpha, epsilon, state, next_state, next2_state, selected, reward)


        if norm_force > 120:

            print("Force interference!")
            wpose.pose.position.z = 0.500
            pub.publish(wpose)

            break #we might terminate too fast because of this, maybe just give neg rew and change state
        #else:
        #    state = next_state

        state = int(next_state/1000)
        statewithforce = getstate(state)
        nosteps += 1
        #should I limit the number of steps?

    #epsilon -= 0.0015
    epsilon = epsilon*0.998
    success = 0
    if int(state/100) == depth and norm_force < 20:
        print("Goal reached, going to initial position")
        success_episodes += 1
        success = 1
        Q[statewithforce]=8000 #idk if this was already updated somewhere. update value of goal with large value ()
    else:
        print("Failed!")

    count += 1
    finish_time = time.time()
    with open('datafile-10ftrew122.txt', 'a') as the_file:
        the_file.write("{},{},{},{},{},{},{},{},{}, \n".format(i, epsilon, start_time, finish_time, success, nosteps, statewithforce, initialx, initialy))

    pd.DataFrame(Q).to_csv("/home/q/catkin_ws/src/hello_ros/scripts/Qmat10ftrew122.csv")
    #np.savetxt("pleaseprint.csv", Q, delimiter=",")

    if success == 1:
        indicator += 1
    else: 
        indicator = 0
    if indicator > 10 and nosteps < 10:
        print ("more than 10 iterations with less than 10 steps to the goal")
        break
pd.DataFrame(Q).to_csv("/home/q/catkin_ws/src/hello_ros/scripts/idkwhyitsnotprintng10ftrew10.csv")


#---------PRINTING-------------------------

#CHANGE THE LOCATION OF THE .CSV FILE
#IF YOUR CYLINDER IS NOT 200MM IN LENGTH, YOU NEED TO ADJUST THE HEIGHT (MAKE IT START CLOSER TO THE ENGINE BLOCK)