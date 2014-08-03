# ----------
# Part Three
#
# Now you'll actually track down and recover the runaway Traxbot. 
# In this step, your speed will be about twice as fast the runaway bot,
# which means that your bot's distance parameter will be about twice that
# of the runaway. You can move less than this parameter if you'd 
# like to slow down your bot near the end of the chase. 
#
# ----------
# YOUR JOB
#
# Complete the next_move function. This function will give you access to 
# the position and heading of your bot (the hunter); the most recent 
# measurement received from the runaway bot (the target), the max distance
# your bot can move in a given timestep, and another variable, called 
# OTHER, which you can use to keep track of information.
# 
# Your function will return the amount you want your bot to turn, the 
# distance you want your bot to move, and the OTHER variable, with any
# information you want to keep track of.
# 
# ----------
# GRADING
# 
# We will make repeated calls to your next_move function. After
# each call, we will move the hunter bot according to your instructions
# and compare its position to the target bot's true position
# As soon as the hunter is within 0.01 stepsizes of the target,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot. 
#
# As an added challenge, try to get to the target bot as quickly as 
# possible. 

from robot import *
from math import *
from matrix import *
import random
import turtle

animate = False

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = .05 * target.distance
#measurement_noise = 0.0
target.set_noise(0.0, 0.0, measurement_noise)
hunter = robot(-10.0, -10.0, 0.0)

gains = [0.1, .5, 0.0015, +0.0005, .0, -.01] # Pdist, Phead, Idist, Ihead, Ddist, Dhead   
#gains = [1.5, 0.0, -0.9] #P, I, D

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    if not OTHER: 
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        distIntList = [0.]
        headIntList = [0.]
        distList = [0.]
        headList = [0.]
        OTHER1 = [[],[]]
        intLimit = [False]
        OTHER = (measurements, hunter_positions, hunter_headings, distIntList, headIntList, distList, headList, OTHER1, intLimit)
    else: 
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings, distIntList, headIntList, distList, headList, OTHER1, intLimit = OTHER 
    
#    nextPos, OTHER, RTarget, center, thetaTarget = estimate_next_pos(target_measurement, OTHER)
    nextPos = target_measurement
    
    '''xError = nextPos[0] - hunter_position[0]
    yError = nextPos[1] - hunter_position[1]
    distError = distance_between(hunter_position, nextPos)
    
#    xInt = xError + distIntList[len(distIntList) - 1]
#    yInt = yError + headIntList[len(headIntList) - 1]
#    OTHER[3].append(xInt)
#    OTHER[4].append(yInt)    
    distInt = distError + distIntList[len(distIntList) - 1]
    OTHER[3].append(distInt)
  
    xDiff = xError - distList[len(distList) - 1]
    yDiff = yError - headList[len(headList) - 1]
    OTHER[5].append(xError)
    OTHER[6].append(yError)
    
    x = (gains[0] + gains[1] * distInt) * xError + gains[2] * xDiff  
    y = (gains[0] + gains[1] * distInt) * yError + gains[2] * yDiff 
    
    newHeading = atan2(y, x)
    turning = angle_trunc(newHeading - hunter_heading)
    distance = sqrt(x**2 + y**2)
    print "distance, turning, distInt ", distance, turning, distInt'''
    
    '''xHunter = hunter_position[0] - center[0]
    yHunter = hunter_position[1] - center[1]
    RHunter = sqrt((xHunter)**2 + (yHunter)**2)
    thetaHunter = atan2(yHunter, xHunter)
    RError = RTarget - RHunter
    thetaError = thetaTarget - thetaHunter
    
    RInt = RError + distIntList[len(distIntList) - 1]
    thetaInt = thetaError + headIntList[len(headIntList) - 1]
    OTHER[3].append(RInt)
    OTHER[4].append(thetaInt)
    
    RDiff = RError - distList[len(distList) - 1]
    thetaDiff = thetaError - headList[len(headList) - 1]
    
    R = gains[0] * RError + gains[2] * RInt + gains[4] * RDiff
    theta = gains[1] * thetaError + gains[3] * thetaInt + gains[5] * thetaDiff 

    x = R * cos(theta)
    y = R * sin(theta)
    
    turning = atan2(y + center[1], x + center[0])
    distance = 1
    turning = pi / 4'''
    
    #consider using x and y as the control variables
    
    heading = atan2(nextPos[1] - hunter_position[1], nextPos[0] - hunter_position[0])
    headError = angle_trunc(heading - hunter_heading) 
    distError = distance_between(hunter_position, nextPos)
  
    if distError < 0.0001 * max_distance:
        intLimit[0] = True
        OTHER[8][0] = True 
    if intLimit[0]:
        distInt = distIntList[len(distIntList) - 1]
        headInt = headIntList[len(headIntList) - 1]
    else:
        distInt = distError + distIntList[len(distIntList) - 1]
        headInt = headError + headIntList[len(headIntList) - 1]
    OTHER[3].append(distInt)
    OTHER[4].append(headInt)
    
    distDiff = - distError + distList[len(distList) - 1]
    headDiff = - headError + headList[len(headList) - 1]
    OTHER[5].append(distError)  
    OTHER[6].append(headError)  

#    print "headError, distError, headInt, distInt: ", headError, distError, headInt, distInt
    
    
    distance = gains[0] * distError + gains[2] * distInt - gains[4] * distDiff
    turning =  gains[1] * headError + gains[3] * headInt + gains[5] * headDiff
#    print "turning, distance: ", turning, distance

    return turning, distance, OTHER
    
def twiddle(tol = 0.0001):

    deltaGains = [.01, .01, .0005, .0005, .001, .001]
    numIterations = 10
    bestError = 0
    for j in range(10):
        bestError += demo_grading(hunter, target, next_move)
    while sum(deltaGains) > tol:
        for i in range(len(gains)):
            gains[i] += deltaGains[i]
            error = 0
            for j in range(numIterations):
                error += demo_grading(hunter, target, next_move)
            if error < bestError:
                bestError = error
                deltaGains[i] *= 1.1
            else:
                gains[i] -= 2 * deltaGains[i]
                error = 0
                for j in range(numIterations):
                    error += demo_grading(hunter, target, next_move)
                if error < bestError:
                    bestError = error
                    deltaGains[i] *= 1.1
                else:
                    gains[i] += deltaGains[i]
                    deltaGains[i] *= 0.9
                    
    
    return gains
    
def estimate_next_pos(measurement, OTHER):

    if len(OTHER[7][0]) < 5:
        OTHER[7][0].append(measurement[0])
        OTHER[7][1].append(measurement[1])
        xy_estimate = measurement
        R = sqrt(measurement[0]**2 + measurement[1]**2)
        curtheta = atan2(measurement[1], measurement[0])
        center = [0, 0]
    else:
        OTHER[7][0].append(measurement[0])
        OTHER[7][1].append(measurement[1])

        x_m = sum(OTHER[7][0]) / len(OTHER[7][0])
        y_m = sum(OTHER[7][1]) / len(OTHER[7][1])
        
        u = []
        v = []
        for i in range(len(OTHER[7][0])):
            u.append(OTHER[7][0][i] - x_m)
            v.append(OTHER[7][1][i] - y_m)
        
        Suv = []
        Suu = [] 
        Svv = [] 
        Suuv = []
        Suvv = []
        Suuu = []
        Svvv = []

        for i in range(len(OTHER[7][0])): 
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
        for i in range(len(OTHER[7][0])):
            res.append(sqrt((OTHER[7][0][i]-center[0])**2 + (OTHER[7][1][i]-center[1])**2))
        R = sum(res) / len(res)

        
        theta = []
        meantheta = 0
        for i in range(len(OTHER[7][0])):
            theta.append(atan2(OTHER[7][1][i] - center[1], OTHER[7][0][i] - center[0]))
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
        
#        curtheta = starttheta + len(theta) * meantheta
        curtheta = starttheta + (len(theta) - 1) * meantheta
        xy_estimate = (center[0] + R * cos(curtheta), center[1] + R * sin(curtheta))        

#        print "i, center, R, meantheta, starttheta: ", i, center, R, meantheta, starttheta

    return xy_estimate, OTHER, R, center, curtheta

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 1.94 * target_bot.distance # 1.94 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # animation
    if animate:
        window = turtle.Screen()
        window.bgcolor('white')
        size_multiplier= 10.0  #change Size of animation
        robot1 = turtle.Turtle()
        robot1.shape('turtle')
        robot1.color('green')
        robot1.resizemode('user')
        robot1.shapesize(0.3, 0.3, 0.3)
        robot2 = turtle.Turtle()
        robot2.shape('circle')
        robot2.color('red')
        robot2.resizemode('user')
        robot2.shapesize(0.3, 0.3, 0.3)
        robot1.penup()
        robot2.penup()

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
#        print "separation: ", separation
        if separation < separation_tolerance:
#            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # animation
        if animate:
            robot1.setheading(hunter_bot.heading*180/pi)
            robot1.goto(hunter_position[0]*size_multiplier, hunter_position[1]*size_multiplier-200)
            robot2.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-200)
    #        robot1.stamp()
    #        robot2.stamp()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)
        
        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1            
#        if ctr >= 1000:
#            print "It took too many steps to catch the target."
            
#    return caught
    if ctr >= 1000:
        error = 10000
    else:
        error = ctr
    return error

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
#    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all 
    the target measurements, hunter positions, and hunter headings over time, but it doesn't 
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables
    
    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER

error = 0
for i in range(100):
    error += demo_grading(hunter, target, next_move)
print error
#print demo_grading(hunter, target, next_move)     
#print twiddle ()




