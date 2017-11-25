grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid) - 1, len(grid[0]) - 1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def get_neighbors(graph, start, delta, cost):
    neighbors = []
    n_rows = len(graph)
    n_cols = len(graph[0])
    for dt in delta:
        i = dt[0] + start[1]
        j = dt[1] + start[2]
        if (i >= n_rows) or (j >= n_cols) or (i < 0) or (j < 0):
            pass
        else:
            if graph[i][j] == 0:
                neighbors.append([start[0] + cost, i, j])
    return neighbors


def search(grid, init, goal, cost):
    # initialize our queue and add cost as a first term
    init.insert(0, 0)
    queue = [init]
    # add an expansion
    expand = [[-1 for i in range(len(grid[0]))] for j in range(len(grid))]
    count = 0
    # while queue is not empty
    while queue:
        # grab first element in the queue
        vertex = queue.pop(0)
        # if we haven't seen this element before
        if grid[vertex[1]][vertex[2]] == 0:
            expand[vertex[1]][vertex[2]] = count
            count += 1
            # set the grid to seen
            grid[vertex[1]][vertex[2]] = 1
            # check if we are at our destination if so return it
            if (vertex[1], vertex[2]) == (goal[0], goal[1]):
                return expand
            neighbors = get_neighbors(grid, vertex, delta, cost)
            queue.extend(neighbors)

    return expand


expand = search(grid, init, goal, cost)

for i in range(len(expand)):
    print(expand[i])
