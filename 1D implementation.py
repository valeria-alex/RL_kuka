import numpy as np
import random
import pandas as pd
import time

#168 degrees
Q = np.zeros((336, 2))
#Parameters
epsilon = 0.8
alpha = 0.6
gamma = 1
goal = 168
count = 1

Q[goal,0] = 100
Q[goal,1] = 100

def available_actions(state):
    actions = Q[state,]
    return actions

def select_action(state,actions):
    if random.uniform(0,1) < epsilon:
        selected = random.randint(0,1)
    else:
        if Q[state][0] < Q[state][1]:
            selected = 1
        elif Q[state][0] > Q[state][1]:
            selected = 0
        else:
            selected = random.randint(0,1)
    return selected

def update(alpha, gamma, reward, state, selected, next_state):
    Q_next = max(Q[next_state,0], Q[next_state,1])
    Q[state,selected] = (1-alpha)*Q[state,selected] + alpha*(reward + gamma*Q_next)

#-------Training-----------------------------
for i in range(1, 2500):
    print(count)
    state = random.randint(0,335)
    while state != goal:

        reward = abs(state-goal)*-0.1

        actions = available_actions(state)
        selected = select_action(state,actions)
            
        if state == 0 and selected == 0:
            next_state = 0
        elif state == 335 and selected == 1:
            next_state = 335
        else:
            if selected == 1:
                next_state = state + 1
            else:
                next_state = state - 1
                    
        update(alpha, gamma, reward, state, selected, next_state)
        epsilon -= 0.01
        state = next_state
    count += 1
Q = Q/np.max(Q)*100    
pd.DataFrame(Q).to_csv("/Users/FJOJe/Desktop/Q-learning.csv")
#--------------------------------------------

#-------Testing------------------------------
state = random.randint(0,335)
steps = [state]

while state != goal:
    if Q[state, 0] < Q[state, 1]:
        next_state = state + 1
    elif Q[state, 0] > Q[state, 1]:
        next_state = state - 1
    else:
        choice = random.randint(0,1)
        if choice == True:
           next_state = state + 1
        else:
           next_state = state - 1

    steps.append(next_state)
    state = next_state
time.sleep(2)
# Print selected sequence of steps
print("Selected path:")
print(steps)
#-----------------------------


            
