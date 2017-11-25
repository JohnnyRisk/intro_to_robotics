# ----------
# User Instructions:
#
# Implement the function optimum_policy2D below.
#
# You are given a car in grid with initial state
# init. Your task is to compute and return the car's
# optimal path to the position specified in goal;
# the costs for each motion are as defined in cost.
#
# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a
# right turn.

forward = [[-1, 0],  # go up
           [0, -1],  # go left
           [1, 0],  # go down
           [0, 1]]  # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']


# EXAMPLE INPUTS:
# grid format:
#     0 = navigable space
#     1 = unnavigable space
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]


init = [4, 3, 0]  # given in the form [row,col,direction]

# direction = 0: up
#             1: left
#             2: down
#             3: right

goal = [2, 0]  # given in the form [row,col]

cost = [2, 1, 40]  # cost has 3 values, corresponding to making




heuristic = [[abs(goal[1] - j) + abs(goal[0] - i) for i in range(len(grid))] for j in range(len(grid[0]))]

# a right turn, no turn, and a left turn

# EXAMPLE OUTPUT:
# calling optimum_policy2D with the given parameters should return
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]
# ----------

# ----------------------------------------
# modify code below
# ----------------------------------------
def get_actions(state):
    x, y, heading, total_cost, path = state[0],state[1],state[2],state[3],state[4]
    actions = []
    for i in range(len(action)):
        movement = (heading + action[i]) % len(forward)
        cost2 = total_cost+cost[i]
        x2 = x + forward[movement][0]
        y2 = y + forward[movement][1]
        heading2 = movement
        path2 = path.copy()
        path2.append((x,y,i))
        if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
            actions.append([x2,y2,heading2, cost2, path2])
    return actions

def optimum_policy2D(grid, init, goal, cost):
    # ----------------------------------------
    # modify code below
    # ----------------------------------------
    movements = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    mem_states={}
    # this is just to start the cost from 0
    init.extend([0,[]])
    queue = [init]
    while queue:
        state = queue.pop(0)
        if (state[0],state[1],state[2]) not in mem_states:
            mem_states[(state[0],state[1],state[2])] = state
            actions = get_actions(state)
            queue.extend(actions)
            #print(state)
        else:
            if mem_states[(state[0],state[1],state[2])][3] > state[3]:
                mem_states[(state[0],state[1],state[2])] = state
                actions = get_actions(state)
                queue.extend(actions)

    return mem_states

# This is all just printing crap
policy =optimum_policy2D(grid,init,goal,cost)
for i in range(len(forward)):
    try:
        a=policy[(2,0,i)]
    except:
        pass

movements = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]

movements[goal[0]][goal[1]] = '*'

for i in a[4]:
    movements[i[0]][i[1]] = action_name[i[2]]

for i in range(len(movements)):
    print(movements[i])