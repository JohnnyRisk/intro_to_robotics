# -------------------
# Background Information
#
# In this problem, you will build a planner that helps a robot
# find the shortest way in a warehouse filled with boxes
# that he has to pick up and deliver to a drop zone.
#
# For example:
#
# warehouse = [[ 1, 2, 3],
#              [ 0, 0, 0],
#              [ 0, 0, 0]]
# dropzone = [2,0]
# todo = [2, 1]
#
# The robot starts at the dropzone.
# The dropzone can be in any free corner of the warehouse map.
# todo is a list of boxes to be picked up and delivered to the dropzone.
#
# Robot can move diagonally, but the cost of a diagonal move is 1.5.
# The cost of moving one step horizontally or vertically is 1.
# So if the dropzone is at [2, 0], the cost to deliver box number 2
# would be 5.

# To pick up a box, the robot has to move into the same cell as the box.
# When the robot picks up a box, that cell becomes passable (marked 0)
# The robot can pick up only one box at a time and once picked up
# it has to return the box to the dropzone by moving onto the dropzone cell.
# Once the robot has stepped on the dropzone, the box is taken away,
# and it is free to continue with its todo list.
# Tasks must be executed in the order that they are given in the todo list.
# You may assume that in all warehouse maps, all boxes are
# reachable from beginning (the robot is not boxed in).

# -------------------
# User Instructions
#
# Design a planner (any kind you like, so long as it works!)
# in a function named plan() that takes as input three parameters:
# warehouse, dropzone, and todo. See parameter info below.
#
# Your function should RETURN the final, accumulated cost to do
# all tasks in the todo list in the given order, which should
# match with our answer. You may include print statements to show
# the optimum path, but that will have no effect on grading.
#
# Your solution must work for a variety of warehouse layouts and
# any length of todo list.
#
# Add your code at line 76.
#
# --------------------
# Parameter Info
#
# warehouse - a grid of values, where 0 means that the cell is passable,
# and a number 1 <= n <= 99 means that box n is located at that cell.
# dropzone - determines the robot's start location and the place to return boxes
# todo - list of tasks, containing box numbers that have to be picked up
#
# --------------------
# Testing
#
# You may use our test function below, solution_check(),
# to test your code for a variety of input parameters.

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

def plan(warehouse, dropzone, todo):
    import heapq
    import numpy as np

    deltas = [[-1, 0],  # go up
              [-1, -1],  # go up-left
              [0, -1],  # go left
              [1, -1],  # go down left
              [1, 0],  # go down
              [1, 1],  # go down right
              [0, 1],  # go right
              [-1, 1]]  # go right up

    costs = [1.0,  # go up
             1.5,  # go up-left
             1.0,  # go left
             1.5,  # go down left
             1.0,  # go down
             1.5,  # go down right
             1.0,  # go right
             1.5]  # go right up

    def get_todo_index(warehouse, todo):
        for i in range(len(warehouse)):
            for j in range(len(warehouse[0])):
                if warehouse[i][j] == todo:
                    return (i, j)
        return (-1, -1)

    def create_heuristic(warehouse, start, deltas, costs):
        # THIS IS JUST DIJIKSTRAS ALGORITHM
        # initilize the hash tables to use
        Q = {}
        prev = {}
        dist = {}
        # format source
        source = (start[0], start[1])

        # this initializes distance from source
        # and previous node
        for i in range(len(warehouse)):
            for j in range(len(warehouse[0])):
                dist[(i, j)] = float('inf')
                prev[(i, j)] = None
                Q[(i, j)] = (i, j)

        dist[source] = 0
        j = 0
        # while Q is not empty we get the vertex in Q with a minimum distance
        while Q:
            u = min(Q, key=dist.get)

            # delete u from Q
            Q.pop(u)
            # each neighbor that is still in Q
            for i, delta in enumerate(deltas):
                neighbor = (u[0] + delta[0], u[1] + delta[1])
                if neighbor in Q:
                    alt = dist[u] + costs[i]
                    if alt < dist[neighbor]:
                        dist[neighbor] = alt
                        prev[neighbor] = u

        return dist, prev

    class PriorityQueue:
        def __init__(self):
            self.elements = []

        def empty(self):
            return len(self.elements) == 0

        def put(self, item, priority):
            heapq.heappush(self.elements, (priority, item))

        def get(self):
            return heapq.heappop(self.elements)[1]

    def a_star_search(graph, heuristic, start, goal, deltas, costs):
        # initialize the cost so far
        cost_so_far = {}
        for i in range(len(graph)):
            for j in range(len(graph[0])):
                cost_so_far[(i, j)] = float('inf')

        start = (start[0], start[1])

        frontier = PriorityQueue()
        frontier.put(start, 0)

        came_from = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break

            for i, delta in enumerate(deltas):
                neighbor = (current[0] + delta[0], current[1] + delta[1])
                new_cost = cost_so_far[current] + costs[i]
                if neighbor in cost_so_far and new_cost < cost_so_far[neighbor]:
                    if graph[neighbor[0]][neighbor[1]] == 0 or neighbor == goal:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + heuristic[neighbor]
                        frontier.put(neighbor, priority)
                        came_from[neighbor] = current

        return came_from, cost_so_far

    warehouse[dropzone[0]][dropzone[1]] = 0
    heuristic, _ = create_heuristic(warehouse, dropzone, deltas, costs)
    total_cost = 0
    for i, td in enumerate(todo):
        x, y = get_todo_index(warehouse, td)
        goal = (x, y)
        path, cost = a_star_search(warehouse, heuristic, dropzone, goal, deltas, costs)
        warehouse[goal[0]][goal[1]] = 0
        total_cost += cost[goal]

    return 2 * total_cost


################# TESTING ##################

# ------------------------------------------
# solution check - Checks your plan function using
# data from list called test[]. Uncomment the call
# to solution_check to test your code.
#
def solution_check(test, epsilon=0.00001):
    answer_list = []

    import time
    start = time.clock()
    correct_answers = 0
    for i in range(len(test[0])):
        user_cost = plan(test[0][i], test[1][i], test[2][i])
        true_cost = test[3][i]
        if abs(user_cost - true_cost) < epsilon:
            print("\nTest case", i + 1, "passed!")
            answer_list.append(1)
            correct_answers += 1
            # print "#############################################"
        else:
            print("\nTest case ", i + 1, "unsuccessful. Your answer ", user_cost, "was not within ", epsilon, "of ", true_cost)
            answer_list.append(0)
    runtime = time.clock() - start
    if runtime > 1:
        print("Your code is too slow, try to optimize it! Running time was: ", runtime)
        return False
    if correct_answers == len(answer_list):
        print("\nYou passed all test cases!")
        return True
    else:
        print("\nYou passed", correct_answers, "of", len(answer_list), "test cases. Try to get them all!")
        return False


# Testing environment
# Test Case 1
warehouse1 = [[1, 2, 3],
              [0, 0, 0],
              [0, 0, 0]]
dropzone1 = [2, 0]
todo1 = [2, 1]
true_cost1 = 9
# Test Case 2
warehouse2 = [[1, 2, 3, 4],
              [0, 0, 0, 0],
              [5, 6, 7, 0],
              ['x', 0, 0, 8]]
dropzone2 = [3, 0]
todo2 = [2, 5, 1]
true_cost2 = 21

# Test Case 3
warehouse3 = [[1, 2, 3, 4, 5, 6, 7],
              [0, 0, 0, 0, 0, 0, 0],
              [8, 9, 10, 11, 0, 0, 0],
              ['x', 0, 0, 0, 0, 0, 12]]
dropzone3 = [3, 0]
todo3 = [5, 10]
true_cost3 = 18

# Test Case 4
warehouse4 = [[1, 17, 5, 18, 9, 19, 13],
              [2, 0, 6, 0, 10, 0, 14],
              [3, 0, 7, 0, 11, 0, 15],
              [4, 0, 8, 0, 12, 0, 16],
              [0, 0, 0, 0, 0, 0, 'x']]
dropzone4 = [4, 6]
todo4 = [13, 11, 6, 17]
true_cost4 = 41

testing_suite = [[warehouse1, warehouse2, warehouse3, warehouse4],
                 [dropzone1, dropzone2, dropzone3, dropzone4],
                 [todo1, todo2, todo3, todo4],
                 [true_cost1, true_cost2, true_cost3, true_cost4]]


# solution_check(testing_suite) #UNCOMMENT THIS LINE TO TEST YOUR CODE
