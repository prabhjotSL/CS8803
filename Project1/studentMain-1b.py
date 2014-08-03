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
from matrix import * # Check the matrix.py tab to see how this works.
import random

# This is the function you have to write. Note that measurement is a 
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be 
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
def estimate_next_pos(measurement, OTHER = None):

    if not OTHER:
        OTHER = [[measurement[0]], [measurement[1]]]
        xy_estimate = measurement
    elif len(OTHER[0]) < 5:
        OTHER[0].append(measurement[0])
        OTHER[1].append(measurement[1])
        xy_estimate = measurement
    else:
        OTHER[0].append(measurement[0])
        OTHER[1].append(measurement[1])
#        from scipy import optimize
#            
#        def residuals(center, x, y):
#            xc, yc = center
#            res = []
#            for i in range(len(x)):
#                res.append(sqrt((x[i]-xc)**2 + (y[i]-yc)**2))
#            return res
#            
#        def f(center, x, y):
#            res = residuals(center, x, y)
#            resMean = sum(res) / len(res)
#            for i in range(len(res)):
#                res[i] -= resMean
#            return res
#            
#        center, ier = optimize.leastsq(f, OTHER[2], args=(OTHER[0], OTHER[1]), ftol=1.e-05, xtol=1.e-05, maxfev=1000)
#        OTHER[2] = center
#            
#        res = residuals(center, OTHER[0], OTHER[1])
#        R = sum(res) / len(res)

        x_m = sum(OTHER[0]) / len(OTHER[0])
        y_m = sum(OTHER[1]) / len(OTHER[1])
        
        u = []
        v = []
        for i in range(len(OTHER[0])):
            u.append(OTHER[0][i] - x_m)
            v.append(OTHER[1][i] - y_m)
        
        Suv = []
        Suu = [] 
        Svv = [] 
        Suuv = []
        Suvv = []
        Suuu = []
        Svvv = []

        for i in range(len(OTHER[0])): 
            Suv.append(u[i]*v[i])
            Suu.append(u[i]*u[i])
            Svv.append(v[i]*v[i])
            Suuv.append(u[i]*u[i]*v[i])
            Suvv.append(u[i]*v[i]*v[i])
            Suuu.append(u[i]*u[i]*u[i])
            Svvv.append(v[i]*v[i]*v[i])
        
        A = matrix([[sum(Suu), sum(Suv)], [sum(Suv), sum(Svv)]])
        B = matrix([[sum(Suuu) + sum(Suvv)], [sum(Svvv) + sum(Suuv)]])
        C = A.inverse() * B
        center = [C.value[0][0] / 2.0 + x_m, C.value[1][0] / 2.0 + y_m]

        res = []
        for i in range(len(OTHER[0])):
            res.append(sqrt((OTHER[0][i]-center[0])**2 + (OTHER[1][i]-center[1])**2))
        R = sum(res) / len(res)

        
        theta = []
        meantheta = 0
        for i in range(len(OTHER[0])):
            theta.append(atan2(OTHER[1][i] - center[1], OTHER[0][i] - center[0]))
            if i > 0:
                diff = theta[i] - theta[i - 1]
                if diff > pi:
                    diff -= 2 * pi
                elif diff < -pi:
                    diff += 2 * pi
                meantheta += diff
        meantheta /= (len(theta) - 1)
        
        starttheta = 0
        for i in range(len(theta)):
            start = (theta[i] - meantheta * i) % pi
            if theta[0] < 0:
                start -= pi
            starttheta += start
        starttheta /= len(theta)
        
        curtheta = starttheta + len(theta) * meantheta
        xy_estimate = (center[0] + R * cos(curtheta), center[1] + R * sin(curtheta))        

        print "i, center, R, meantheta, starttheta: ", i, center, R, meantheta, starttheta

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
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
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
    broken_robot.shapesize(0.3, 0.3, 0.3)
    measured_broken_robot = turtle.Turtle()
    measured_broken_robot.shape('circle')
    measured_broken_robot.color('red')
    measured_broken_robot.resizemode('user')
    measured_broken_robot.shapesize(0.3, 0.3, 0.3)
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.3, 0.3, 0.3)
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
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
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

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that 
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER 
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
#test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
#test_target = robot(0, 0, pi, 2*pi / 34.0, 1.5)
#test_target = robot()
test_target = robot(-pi, -pi, -pi, -1.1*2*pi/34.0, 1.1)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

demo_grading(estimate_next_pos, test_target)




