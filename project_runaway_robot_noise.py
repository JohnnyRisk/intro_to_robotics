# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position.
#
# ----------
# GRADING
#
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import *  # Check the matrix.py tab to see how this works.
import random


# This is the function you have to write. Note that measurement is a
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
def sign(x):
    if x >= 0:
        sign = 1
    else:
        sign = 0
    return sign


def dist_heading(point1, point2):
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    hypot = distance_between(point2, point1)

    heading = (1 - sign(dy)) * (360 - degrees(acos(dx / hypot))) + sign(dy) * (degrees(acos(dx / hypot)))
    return hypot, heading


def change_heading(heading1, heading2):
    if abs(heading1 - heading2) > 180:
        if heading1 < heading2:
            new_heading = (360 - heading2) - heading1
        else:
            new_heading = (360 - heading1) + heading2
    else:
        new_heading = heading2 - heading1
    return new_heading


def test_change_heading():
    assert change_heading(352, 2) == 10
    assert change_heading(150, 90) == -60
    assert change_heading(270, 90) == -180
    print("heading change test passed")


def test_dist_heading():
    hypot, heading = dist_heading([1, 1], [0, 1])
    assert hypot == 1
    assert heading == 180
    hypot, heading = dist_heading([1, 1], [-1, -1])
    assert hypot == 2 * sqrt(2)
    assert heading == 225
    print('all test_dist and heading passed')


def test_predict_coordinate():
    assert predict_coordinate([1, 1], 90, 90, 1) == [0.0, 1.0]
    assert predict_coordinate([0, 0], 90, 45, sqrt(2)) == [-1.0, 1.0]
    assert predict_coordinate([0, -1], 315, 90, sqrt(2)) == [1.0, 0.0]
    print('test_predicted coordinates pass')


def predict_coordinate(point, heading, dheading, ddist):
    new_heading = heading + dheading
    dy = ddist * sin(new_heading * (2 * pi) / 360)
    dx = ddist * cos(new_heading * (2 * pi) / 360)
    predicted_xy = [round(point[0] + dx, 3), round(point[1] + dy, 3)]
    return predicted_xy


def update_guassian(mu_old, sigma_old, obs, var_obs):
    mu_prime = (var_obs * mu_old + sigma_old * obs) / (sigma_old + var_obs)
    sigma_prime = sigma_old * var_obs / (sigma_old + var_obs)
    return mu_prime, sigma_prime


def estimate_next_pos(measurement, OTHER=None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    ## initialize this for the first time
    if not OTHER:
        OTHER = [[], [], []]
    OTHER[0].append(measurement)
    xy_estimate = measurement

    ## the next time we initialize the distance
    if len(OTHER[0]) == 2:
        sigma_dist = 1
        point1 = OTHER[0][-2]
        point2 = OTHER[0][-1]
        dist, heading = dist_heading(point1, point2)
        OTHER[1] = [dist, sigma_dist]
        OTHER[2] = [heading]
    # This time we initialize the change in heading
    elif len(OTHER[0]) == 3:
        # get the points
        point1 = OTHER[0][-2]
        point2 = OTHER[0][-1]
        # get the stuff to update distance
        mu_old = OTHER[1][0]
        sigma_old = OTHER[1][1]
        var_obs = 1
        # get old heading
        old_heading = OTHER[2][0]
        heading_sigma = 5

        # get new heading and distance
        dist, new_heading = dist_heading(point1, point2)
        # update based on uncertainty in our guassians
        new_dist_mu, new_dist_sigma = update_guassian(mu_old, sigma_old, dist, var_obs)
        # get new change in heading
        dheading = change_heading(old_heading, new_heading)
        OTHER[1] = [new_dist_mu, new_dist_sigma]
        OTHER[2] = [new_heading, dheading, heading_sigma]
        xy_estimate = predict_coordinate(point2, new_heading, dheading, new_dist_mu)

    elif len(OTHER[0]) > 3:
        # get the points
        point1 = OTHER[0][-2]
        point2 = OTHER[0][-1]
        # get the stuff to update distance
        mu_old = OTHER[1][0]
        sigma_old = OTHER[1][1]
        var_obs = 1.0
        # get old heading
        old_heading = OTHER[2][0]
        old_dheading = OTHER[2][1]
        old_heading_sigma = OTHER[2][2]
        heading_sigma = 5.0

        # get new heading and distance
        dist, new_heading = dist_heading(point1, point2)
        # update based on uncertainty in our guassians
        new_dist_mu, new_dist_sigma = update_guassian(mu_old, sigma_old, dist, var_obs)
        # get new change in heading
        dheading = change_heading(old_heading, new_heading)
        # update change in heading based on uncertainty
        dheading_mu, dheading_sigma = update_guassian(old_dheading, old_heading_sigma, dheading, heading_sigma)
        OTHER[1] = [new_dist_mu, new_dist_sigma]
        OTHER[2] = [new_heading, dheading_mu, dheading_sigma]
        xy_estimate = predict_coordinate(point2, new_heading, dheading_mu, new_dist_mu)
    else:
        print('ERROR')
    return xy_estimate, OTHER


# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any
# information that you want.
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER=None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print("You got it right! It took you ", ctr, " steps to localize.")
            localized = True
        if ctr == 1000:
            print("Sorry, it took you too many steps to localize the target.")
    return localized


# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER=None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that
    position, so it always guesses that the first position will be the next."""
    if not OTHER:  # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER
    return xy_estimate, OTHER

def demo_grading1(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    #For Visualization
    import turtle    #You need to run this locally to use the turtle module
    window = turtle.Screen()
    window.bgcolor('white')
    size_multiplier= 25.0  #change Size of animation
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.1, 0.1, 0.1)
    measured_broken_robot = turtle.Turtle()
    measured_broken_robot.shape('circle')
    measured_broken_robot.color('red')
    measured_broken_robot.resizemode('user')
    measured_broken_robot.shapesize(0.1, 0.1, 0.1)
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.1, 0.1, 0.1)
    prediction.penup()
    broken_robot.penup()
    measured_broken_robot.penup()
    #End of Visualization
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print("You got it right! It took you ", ctr, " steps to localize.")
            localized = True
        if ctr == 1000:
            print("Sorry, it took you too many steps to localize the target.")
        #More Visualization
        measured_broken_robot.setheading(target_bot.heading*180/pi)
        measured_broken_robot.goto(measurement[0]*size_multiplier, measurement[1]*size_multiplier-200)
        measured_broken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(position_guess[0]*size_multiplier, position_guess[1]*size_multiplier-200)
        prediction.stamp()
        #End of Visualization
    return localized

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2 * pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)
demo_grading1(estimate_next_pos, test_target)
demo_grading(naive_next_pos, test_target)




