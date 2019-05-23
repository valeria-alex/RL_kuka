import random
import numpy as np
import pandas as pd

Q = np.zeros((10000, 8))

epsilon = 1
alpha = 0.5
gamma = 1
count = 1

goal = 7575

def available_actions(state):
    LT = state - 100 - 1
    RT = state - 100 + 1
    LB = state + 100 - 1
    RB = state + 100 + 1
    L = state - 1
    R = state + 1
    T = state - 100
    B = state + 100
    return(LT, RT, LB, RB, L, R, T, B)

def select_action(): #The greedy function is not functional as of now.
    if random.uniform(0,1) < epsilon:
        select = random.randint(0,7)
    elif max(abs(Q[state])) == 0:
        select = random.randint(0,7)
    else:
        select = np.argmax(Q[state])
    return(select)

def update(alpha, gamma, epsilon, state, next_state, selected):
    Q_next = max(Q[next_state])
    Q[state][selected] = (1-alpha)*Q[state][selected]+ alpha*(reward + gamma*Q_next)

#------------TRAINING------------------
for i in range(1,50000):
    print(count)
    state = random.randint(0,9999)
    while state != goal:
        actions = available_actions(state)
        selected = select_action()

        next_state = actions[selected]
        
        if state < 0 or next_state < 0 or state > 9999 or next_state > 9999:
            next_state = state
        elif state % 100 == 0 and (next_state - 99) % 100 == 0 or (state - 99) % 100 == 0 and next_state % 100 == 0:
            next_state = state

        if next_state == goal:
            reward = 100
        else:
            reward = -1*(int(abs(goal-state) / 100) + (abs((goal % 100)-(state % 100))))
        update(alpha, gamma, epsilon, state, next_state, selected)
        state = next_state
    epsilon = epsilon*0.99995
    count += 1

#-----------PRINTING--------------------
test = []
for i in range(0,10000):
    maxi = max(Q[i])
    test.append(maxi)

test = np.asarray(test).reshape((100,100))

pd.DataFrame(Q).to_csv("/Users/FJOJe/Desktop/2D_raw.csv")
pd.DataFrame(test).to_csv("/Users/FJOJe/Desktop/2D_test.csv")


#-----------TESTING---------------------
state = random.randint(0,9999)
step = [state]
while state != goal:
    actions = available_actions(state)
    selected = np.argmax(Q[state])

    next_state = actions[selected]

    if state < 0 or next_state < 0 or state > 9999 or next_state > 9999:
        next_state = state
    elif state % 100 == 0 and next_state - 99 % 100 == 0 or state - 99 % 100 == 0 and next_state % 100 == 0:
        next_state = state
    else:
        pass
    state = next_state
    step.append(state)
print(step)
