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

cost = [2, 1, 20]  # cost has 3 values, corresponding to making

value = [[abs(goal[0] - j) + abs(goal[1] - i) for i in range(len(grid))] for j in range(len(grid[0]))]
# for i in value:
#     print(i)

def get_actions(state,total_cost):
    x, y, heading = state[0],state[1],state[2]
    actions = []
    for i in range(len(action)):
        movement = (heading + action[i]) % len(forward)
        cost2 = total_cost+cost[i]
        x2 = x + forward[movement][0]
        y2 = y + forward[movement][1]
        heading2 = movement
        if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
            actions.append([x2,y2,heading2, cost2])
    return actions

actions = get_actions([2, 3, 0],0)

def solve_path(state):
    actions = get_actions(state,0)
    if not actions:
        return float('inf')
    for act in actions:
        if act[0]==goal[0] and act[1]==goal[1]:
            return act[3]
        else:
            solve_path(act)


    return min(actions)