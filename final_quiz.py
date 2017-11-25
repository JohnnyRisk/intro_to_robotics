p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'green', 'red', 'green', 'red']
Z = 'red'
pHit = 0.9
pMiss = 0.1

def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for i in range(len(p)):
        q[i]=q[i]/s
    return q

def move(p):
    q = [0,0,0,0,0]
    for i in range(len(p)):
        if i== len(p)-1:
            q[i] += p[i]
        else:
            q[i+1] += p[i]
    return q

q = sense(p,Z)
print(q)
q = move(q)
print(q)

print((q[2]+q[4])*0.9 + (q[1]+q[3])*0.1)

def update(mean1, var1, mean2, var2):
    new_mean = 1 / (var1 + var2) * (mean1*var2 + mean2*var1)
    new_var = var1 * var2 / (var1 + var2)
    return [new_mean, new_var]

print(update(1.,1.,3., 1.))


warehouse = [[1, 2, 3],
             [0, 0, 0],
             [0, 0, 0]]
dropzone = [2, 0]
todo = [2, 1]


# ------------------------------------------
# plan - Returns cost to take all boxes in the todo list to dropzone
#
# ----------------------------------------
# modify code below
# ----------------------------------------
import numpy as np
delta = [[-1, 0],  # go up
         [-1, -1],  # go up-left
         [0, -1],  # go left
         [1, -1],  # go down-left
         [1, 0],  # go down
         [1, 1],  # go down-right
         [0, 1],  # go right
         [-1, 1]]  # go up-right
def get_todo_index(warehouse,todo):
    warehouse = np.array(warehouse)
    i, j = np.where(warehouse==todo)
    return [i[0], j[0]]
def get_heuristic(warehouse,todo,dropzone):
    return

print(get_todo_index(warehouse, todo[0]))