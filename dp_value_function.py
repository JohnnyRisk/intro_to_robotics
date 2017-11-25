# ----------
# User Instructions:
# 
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal. 
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']
def get_neighbors(graph, start, delta, cost):
    neighbors = []
    x = start[0]
    y = start[1]
    for i in range(len(delta)):
        x2 = x + delta[i][0]
        y2 = y + delta[i][1]
        if x2 >= 0 and x2 < len(graph) and y2 >= 0 and y2 < len(graph[0]):
            neighbors.append([start[0] + cost, i, j])
    return neighbors

def compute_value(grid,goal,cost):
    # ----------------------------------------
    # insert code below
    # ----------------------------------------
    value = [[0 if i ==0 else 99 for i in row] for row in grid]
    visited = [[0 for _ in row] for row in grid]
    visited[goal[0]][goal[1]] = 1
    open_vert = [goal]
    while open_vert:
        vertex = open_vert.pop()
        x = vertex[0]
        y = vertex[1]
        for i in range(len(delta)):
            x2 = x + delta[i][0]
            y2 = y + delta[i][1]
            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                if visited[x2][y2] == 0 and grid[x2][y2] == 0:
                    open_vert.append([x2, y2])
                    visited[x2][y2] = 1
                    value[x2][y2] = value[x][y] + cost

    # make sure your function returns a grid of values as 
    # demonstrated in the previous video.
    return value
value = compute_value(grid,goal,cost)
for i in range(len(value)):
    print(value[i])

