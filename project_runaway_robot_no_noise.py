# ----------
# Background
#
# A robotics company named Trax has created a line of small self-driving robots
# designed to autonomously traverse desert environments in search of undiscovered
# water deposits.
#
# A Traxbot looks like a small tank. Each one is about half a meter long and drives
# on two continuous metal tracks. In order to maneuver itself, a Traxbot can do one
# of two things: it can drive in a straight line or it can turn. So to make a
# right turn, A Traxbot will drive forward, stop, turn 90 degrees, then continue
# driving straight.
#
# This series of questions involves the recovery of a rogue Traxbot. This bot has
# gotten lost somewhere in the desert and is now stuck driving in an almost-circle: it has
# been repeatedly driving forward by some step size, stopping, turning a certain
# amount, and repeating this process... Luckily, the Traxbot is still sending all
# of its sensor data back to headquarters.
#
# In this project, we will start with a simple version of this problem and
# gradually add complexity. By the end, you will have a fully articulated
# plan for recovering the lost Traxbot.
#
# ----------
# Part One
#
# Let's start by thinking about circular motion (well, really it's polygon motion
# that is close to circular motion). Assume that Traxbot lives on
# an (x, y) coordinate plane and (for now) is sending you PERFECTLY ACCURATE sensor
# measurements.
#
# With a few measurements you should be able to figure out the step size and the
# turning angle that Traxbot is moving with.
# With these two pieces of information, you should be able to
# write a function that can predict Traxbot's next location.
#
# You can use the robot class that is already written to make your life easier.
# You should re-familiarize yourself with this class, since some of the details
# have changed.
#
# ----------
# YOUR JOB
#
# Complete the estimate_next_pos function. You will probably want to use
# the OTHER variable to keep track of information about the runaway robot.
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
from robot import *
from math import *
from matrix import *
import random


# This is the function you have to write. The argument 'measurement' is a
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


def estimate_next_pos(measurement, OTHER=None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    if not OTHER:
        OTHER = [measurement]
    else:
        OTHER.append(measurement)
    if len(OTHER) < 3:  # this is the first measurement
        xy_estimate = OTHER[-1]
    else:
        point1 = OTHER[-3]
        point2 = OTHER[-2]
        point3 = OTHER[-1]
        old_dist, old_heading = dist_heading(point1, point2)
        new_dist, new_heading = dist_heading(point2, point3)
        ddist = (old_dist + new_dist) / 2

        dheading = change_heading(old_heading, new_heading)
        xy_estimate = predict_coordinate(point3, new_heading, dheading, ddist)
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
    while not localized and ctr <= 10:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print("You got it right! It took you ", ctr, " steps to localize.")
            localized = True
        if ctr == 10:
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


# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2 * pi / 34.0, 1.5)
test_target.set_noise(0.0, 0.0, 0.0)

demo_grading(estimate_next_pos, test_target)