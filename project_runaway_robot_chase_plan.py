# ----------
# Part Four
#
# Again, you'll track down and recover the runaway Traxbot.
# But this time, your speed will be about the same as the runaway bot.
# This may require more careful planning than you used last time.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time.
#
# ----------
# GRADING
#
# Same as part 3. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random
import numpy as np
from numpy import dot, sum, tile
from numpy.linalg import inv


def gauss_pdf(X, M, S):
    if M.shape()[1] == 1:
        DX = X - tile(M, X.shape()[1])
        E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
        E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(np.linalg.det(S))
        P = exp(-E)
    elif X.shape()[1] == 1:
        DX = tile(X, M.shape()[1]) - M
        E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
        E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(np.linalg.det(S))
        P = exp(-E)
    else:
        DX = X - M
        E = 0.5 * dot(DX.T, dot(inv(S), DX))
        E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(np.linalg.det(S))
        P = exp(-E)
    return (P[0], E[0])


class kahlman_filter:
    def __init__(self, X, P, F, U, Z, H, R):
        """This function is called when you create a new robot. It sets some of
        the attributes of the robot, either to their default values or to the values
        specified when it is created."""
        self.X = X #prediction
        self.P = P #error cov matrix
        self.F = F # function matrix transforms
        self.U = U # external movement matrix
        self.Z = Z # measurements
        self.H = H # measurement function matrix
        self.R = R # measurement noise

    def kf_update(self,X, P, Y, H, R):
        IM = np.dot(H, X)
        IS = R + np.dot(H, np.dot(P, H.T))
        K = np.dot(P, np.dot(H.T, inv(IS)))
        X = X + np.dot(K, (Y - IM))
        P = P - np.dot(K, np.dot(IS, K.T))
        LH = gauss_pdf(Y, IM, IS)
        return (X, P, K, IM, IS, LH)


    def kf_predict(self,X, P, A, Q, B, U):
        X = np.dot(A, X) + np.dot(B, U)
        P = np.dot(A, np.dot(P, A.T)) + Q
        return X, P


def make_plan(hunter_position, target_measurement, target_heading, distance, d_heading, max_distance):
    solved = False
    i = 1
    while not solved:
        next_target_position, next_target_heading = calc_next_position(
            target_measurement, target_heading, distance, d_heading)
        distance_to_position = distance_between(hunter_position, next_target_position)
        if distance_to_position / (max_distance * i) <= 1:
            solved = True
        else:
            i += 1
            target_measurement = next_target_position
            target_heading = next_target_heading
    return next_target_position


def calc_next_position(target_measurement, target_heading, distance, d_heading):
    x, y = target_measurement[0], target_measurement[1]
    new_heading = target_heading + d_heading
    dy = distance * sin(new_heading)
    dx = distance * cos(new_heading)
    predicted_xy = [round(x + dx, 3), round(y + dy, 3)]
    return predicted_xy, new_heading

def update_guassian(mu_old,sigma_old, obs, var_obs):
    mu_prime = (var_obs*mu_old + sigma_old*obs) / (sigma_old + var_obs)
    sigma_prime = sigma_old * var_obs / (sigma_old + var_obs)
    return mu_prime, sigma_prime

def dist_head_estimates(measurements, sigma_dist_measurement=5, sigma_d_heading_measurement=10):
    for k in range(len(measurements) - 1):
        if k == 0:
            mu_dist = distance_between(measurements[k], measurements[k + 1])
            heading1 = get_heading(measurements[k], measurements[k + 1])
        if k == 1:
            new_dist = distance_between(measurements[k], measurements[k + 1])
            new_heading = get_heading(measurements[k], measurements[k + 1])
            mu_d_heading = new_heading - heading1
            ## update the expectation of mu_dist
            mu_dist, sigma_dist = update_guassian(
                mu_dist, sigma_dist_measurement, new_dist, sigma_dist_measurement)
            old_heading = new_heading
        if k == 2:
            new_dist = distance_between(measurements[k], measurements[k + 1])
            new_heading = get_heading(measurements[k], measurements[k + 1])
            new_d_heading = new_heading - old_heading
            old_heading = new_heading
            ## update the expectation of mu but this time use our new sigma
            mu_dist, sigma_dist = update_guassian(
                mu_dist, sigma_dist, new_dist, sigma_dist_measurement)
            ## update the expectation of mu_d_heading
            mu_d_heading, sigma_d_heading = update_guassian(
                mu_d_heading, sigma_d_heading_measurement, new_d_heading, sigma_d_heading_measurement)
        if k >= 3:
            new_dist = distance_between(measurements[k], measurements[k + 1])
            new_heading = get_heading(measurements[k], measurements[k + 1])
            new_d_heading = new_heading - old_heading
            old_heading = new_heading

            mu_dist, sigma_dist = update_guassian(
                mu_dist, sigma_dist, new_dist, sigma_dist_measurement)

            ## update the expectation of mu_d_heading but this time use our new sigma
            mu_d_heading, sigma_d_heading = update_guassian(
                mu_d_heading, sigma_d_heading, new_d_heading, sigma_d_heading_measurement)

    return mu_dist, mu_d_heading

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):
    # This function will be called after each time the target moves.

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.

    if not OTHER:  # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        plan = []
        hunter_params = []
        OTHER = [measurements, hunter_positions, hunter_headings, plan,
                 hunter_params]  # now I can keep track of history
    else:  # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings, plan, hunter_params = OTHER  # now I can always refer to these variables

    # now we start taking steps
    # Step One: we just go as close as we can to the starting position of the hunted
    if len(measurements) == 1:
        heading_to_target = get_heading(hunter_position, target_measurement)
        heading_difference = angle_trunc(heading_to_target - hunter_heading)
        turning = heading_difference  # turn towards the target
        distance = min(distance_between(hunter_position, target_measurement), max_distance)

    # Step Two: Now we know the distance it travels but not the heading, so extrapolate and try to go
    # as close to the targets next point if it didnt turn
    elif len(measurements) == 2:
        # find the distance it will travel
        dist = distance_between(measurements[-2], measurements[-1])
        # find the heading the target is going
        target_heading = get_heading(measurements[-2], measurements[-1])
        # now we need to find the x,y of the prediction given the dist
        dx = dist * cos(target_heading)
        dy = dist * sin(target_heading)
        pred_target = (measurements[-1][0] + dx, measurements[-1][1] + dy)

        # finally we get our position relative to the pred and move to that spot
        heading_to_target = get_heading(hunter_position, pred_target)
        heading_difference = angle_trunc(heading_to_target - hunter_heading)
        turning = heading_difference  # turn towards the target
        distance = min(distance_between(hunter_position, pred_target), max_distance)

        # we add the hunter params for later use
        hunter_params = [dist, target_heading]
        OTHER[4] = hunter_params
    # Step Three: we will finally know the system determinalistically so we can find the optimal path
    # we will find the turning angle, create a plan, then execute the plan.
    elif len(measurements) >= 3:
        # get the direction the target just moved and calculate how much its steering angle is
        target_heading = get_heading(measurements[-2], measurements[-1])
        dist = np.mean([distance_between(measurements[j], measurements[j+1]) for j in range(len(measurements)-1)])
        d_heading = np.mean(np.diff([get_heading(measurements[j], measurements[j+1]) for j in range(len(measurements)-1)]))
        dist, d_heading = dist_head_estimates(measurements)
        print("heading: {}".format(d_heading))
        print("distance: {}".format(dist))
        # make a plan
        plan = make_plan(hunter_position, target_measurement, target_heading, dist, d_heading, max_distance)
        # store the plan (it is just the destination we will hit the target)
        OTHER[3] = plan
        heading_to_target = get_heading(hunter_position, plan)
        heading_difference = angle_trunc(heading_to_target - hunter_heading)
        print('hunter_position: {}'.format(hunter_position))
        print('current_target_location: {}'.format(target_measurement))
        print('planned_target: {}'.format(plan))
        print('heading_to_plan: {}'.format(heading_to_target))
        print('turning: {}'.format(heading_difference))
        turning = heading_difference  # turn towards the target
        distance = min(distance_between(hunter_position, plan), max_distance)
    else:
        print('ERROR')

    return turning, distance, OTHER


def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER=None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance  # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance  # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print("You got it right! It took you ", ctr, " steps to catch the target.")
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance,
                                                 OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1
        if ctr >= 1000:
            print("It took too many steps to catch the target.")
    return caught


def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi


def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading
def demo_grading1(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 1.94 * target_bot.distance # 1.94 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0 #change Size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print("You got it right! It took you ", ctr, " steps to catch the target.")
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        #Visualize it
        measuredbroken_robot.setheading(target_bot.heading*180/pi)
        measuredbroken_robot.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        chaser_robot.setheading(hunter_bot.heading*180/pi)
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
        #End of visualization
        ctr += 1
        if ctr >= 1000:
            print("It took too many steps to catch the target.")
    return caught

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all
    the target measurements, hunter positions, and hunter headings over time, but it doesn't
    do anything with that information."""
    if not OTHER:  # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings)  # now I can keep track of history
    else:  # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER  # now I can always refer to these variables

    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference  # turn towards the target
    distance = max_distance  # full speed ahead!
    return turning, distance, OTHER


target = robot(0.0, 10.0, 0.0, 2 * pi / 30, 1.5)
measurement_noise = 0.5*target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

#print(demo_grading(hunter, target, next_move))
print(demo_grading1(hunter, target, next_move))





