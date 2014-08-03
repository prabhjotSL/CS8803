'''###########################################################################
# Summer 2014 CS8803: Artifical Intelligence for Robotics
# Final Project: HexBug Tracker
# Author: William Clay Flannigan
# See readme.md for description and usage
###########################################################################'''

import matplotlib.pyplot as plt
import numpy as np
import json
import sys
from matplotlib import animation
from math import cos, sin, atan2, pi, sqrt

# CONSTANTS
MAXVEL          = 50    # Maximum forward velocity limit for motion model
MAXANGVEL       = 0.03  # Maximum angular velocity limit for motion model
WALLPERCENTILE  = 1     # Percential used to determine wall locations
MAXPLOTPOINTS   = 100   # Number of points to plot
NUMPREDICTIONS  = 63    # Number of video frames to extrapolate
NUMRUNS         = 1     # If > 1, loop through data and execute n times

#
# Class point
# Container for state information of the hexbug at each frame
#
class point:
    
    def __init__(self):
        self.frame = 0
        self.pos = [0, 0]
        self.head = 0.0
        self.vel = 0.0
        self.angVel = 0.0
        self.accel = 0.0
        self.angAccel = 0.0
        self.nearWall = False
        
    def __getitem__(self, key):
        return self.pos[key]

    def __setitem__(self, key, value):
        self.pos[key] = value
#
# Class data
# Includes file I/O, plotting, and utility functions for HexBug centroid data
#        
class data:
    
    def __init__(self):
        self.fileName = ''                  # input file
        self.points = []                    # array of HexBug states
        self.means = [0, 0, 0, 0, 0]        # average x, y, head, vel, angVel
        self.stds = [0, 0, 0, 0, 0]         # standard deviation x, y, head, vel, angVel
        self.middleMeans = [0, 0, 0, 0, 0]  # averages of points not near walls
        self.middleStds = [0, 0, 0, 0, 0]   # standard deviation of points not near walls
        self.wall = [0, 0, 0, 0]            # X-, X+, Y-, Y+
                
    def __getitem__(self, key):
        return self.points[key]

    def __setitem__(self, key, value):
        self.points[key] = value        
        
    def __len__(self):
        return len(self.points)
    
    # import data from input file    
    def readFile(self, fileName):
        try:
            self.fileName = fileName            
            f = open(fileName)
            pos = json.load(f)
            for i in range(len(pos)):
                p = point()
                p.pos = pos[i]
                p.frame = i
                self.points.append(p)

        except IOError as e:
            print "I/O error: {0}: {1}".format(e.strerror, fileName)
            raise
        except ValueError:
            print "Could not read data as valid JSON array"
            f.close()
            raise
        except:
            print "Unexpected error:", sys.exc_info()[0]
            f.close()
            raise
        else:
            f.close()
    
    # return a copy of x values of hexbug centroid    
    def x(self):
        x = []
        for i in range(len(self)):
            x.append(self[i][0])
        return x
    
    # return a copy of y values of hexbug centroid
    def y(self):
        y = []
        for i in range(len(self)):
            y.append(self[i][1])
        return y        
    
    # calculate the hexbug orientation from differential motion
    def heading(self):
        h= [0]        
        for i in range(1, len(self)):
            self[i].head = atan2(self[i][1] - self[i-1][1], self[i][0] - self[i-1][0])
            h.append(self[i].head)
        return h
        
    # calculate forward velocity from differential positions
    def velocity(self):
        v = [0]
        for i in range(1, len(self)):
            self[i].vel = self.dist(self[i], self[i-1])
            v.append(self[i].vel)
        return v

    # caluclate forward acceleration from differential velocities
    def acceleration(self):
        a = [0]
        v = self.velocity()
        for i in range(1, len(v)):
            self[i].accel = v[i] - v[i-1]
            a.append(self[i].accel)
        return a

    # calculate angular velocity from differential headings
    def angularVel(self):
        a = [0]
        h = self.heading()
        for i in range(1, len(h)):
            self[i].angVel = h[i] - h[i-1]
            a.append(self[i].angVel)
        return a
    
    # calculate angular acceleration from differential angual velocities
    def angularAccel(self):
        a = [0]
        v = self.angularVel()
        for i in range(1, len(v)):
            self[i].angAccel = v[i] - v[i-1]
            a.append(self[i].angAccel)
        return a

    # in-place interpolation of invalid centroids and outlier velocities
    def interpolate(self):
        for i in range(1, len(self) - 1):
            cnt = 0
            while cnt < len(self) and \
                  (self[i+cnt][0] == -1 or \
                  self.dist(self[i+cnt], self[i+cnt-1]) > MAXVEL):    
                cnt += 1
            for j in range(cnt):
                self[i+j][0] = self[i-1][0] + (j + 1) * (self[i+cnt][0] - self[i-1][0]) / (cnt + 1)
                self[i+j][1] = self[i-1][1] + (j + 1) * (self[i+cnt][1] - self[i-1][1]) / (cnt + 1)
    
    # Euclidian distance between two positions
    def dist(self, a, b):
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    # average of a list
    def mean(self, a): 
        return sum(a) * 1.0 / len(a)
    
    # standard deviation of a list    
    def std(self, a):
        avg = self.mean(a)
        return sqrt(sum((x-avg)**2 for x in a) / len(a))
    
    # calculate the mean and standard deviation of all points and those far from a wall
    def statistics(self):
        
        # all points
        self.means[0] = self.mean(self.x())
        self.means[1] = self.mean(self.y())
        self.means[2] = self.mean(self.heading())
        self.means[3] = self.mean(self.velocity())
        self.means[4] = self.mean(self.angularVel())
        self.stds[0] = self.std(self.x())
        self.stds[1] = self.std(self.y())
        self.stds[2] = self.std(self.heading())
        self.stds[3] = self.std(self.velocity())
        self.stds[4] = self.std(self.angularVel())

        # points not near walls (middle points)
        self.walls()
        middlePoints = []
        for p in self.points:
            if not p.nearWall:
                middlePoints.append([p[0], p[1], p.head, p.vel, p.angVel])
        m = np.array(middlePoints)
        for i in range(len(self.middleMeans)):
            self.middleMeans[i] = self.mean(m.T[i])
            self.middleStds[i] = self.std(m.T[i])
            
        return self.means, self.stds, self.middleMeans, self.middleStds
    
    # calculation of wall locations and in-place labeling of points near walls  
    def walls(self):
        x = self.x()
        x.sort()
        y = self.y()
        y.sort()
        
        # walls are defined by locations of percentile points
        self.wall = [x[int(len(self)*WALLPERCENTILE/100)], 
                     x[int(len(self)*(100-WALLPERCENTILE)/100)], 
                     y[int(len(self)*WALLPERCENTILE/100)], 
                     y[int(len(self)*(100-WALLPERCENTILE)/100)]]
        for i in range(len(self)):
            if self[i][0] < self.wall[0] or \
               self[i][0] > self.wall[1] or \
               self[i][1] < self.wall[2] or \
               self[i][1] > self.wall[3]:
                self[i].nearWall = True
        
        return self.wall
    
    # plot the state variables    
    def plot(self):              
        wallPointsX = []
        wallPointsY = []
        wallPointsVel = []
        wallPointsAngVel = []
        wallPointsAccel = []
        wallPointsAngAccel = []
        for point in self.points:
            if point.nearWall:
                wallPointsX.append(point[0])
                wallPointsY.append(point[1])
                wallPointsVel.append(point.vel)
                wallPointsAngVel.append(point.angVel)
                wallPointsAccel.append(point.accel)
                wallPointsAngAccel.append(point.angAccel)
        pos = plt.figure()
        pos.add_subplot(1, 1, 1)
        
        U = []
        V = []
        for point in self.points:
            U.append(point.vel*cos(point.head))
            V.append(point.vel*sin(point.head))
        plt.quiver(self.x(), self.y(), U, V)        
        plt.title('Position')
        plt.ylabel('Y')
        plt.xlabel('X')
        
        head = plt.figure()
        head.add_subplot(1, 1, 1)
        plt.plot(self.heading()[0:MAXPLOTPOINTS])        
        plt.title('Heading')
        plt.ylabel('Angle')
        plt.xlabel('Frame')
        
        vel = plt.figure()
        vel.add_subplot(1, 2, 1)
        plt.plot(self.velocity()[0:MAXPLOTPOINTS])
        plt.title('Velocity')
        plt.ylabel('Pixel/Frame')
        plt.xlabel('Frame')
        
        vel.add_subplot(1, 2, 2)
        plt.hist(self.velocity(), bins = 50)
        plt.hist(wallPointsVel, bins = 50, color='y')
        plt.title('Velocity')
        plt.ylabel('Count')
        plt.xlabel('Pixel/Frame')
        
        accel = plt.figure()
        accel.add_subplot(1, 2, 1)
        plt.plot(self.acceleration()[0:MAXPLOTPOINTS])
        plt.title('Acceleration')
        plt.ylabel('Pixel/Frame^2')
        plt.xlabel('Frame')
        
        accel.add_subplot(1, 2, 2)
        plt.hist(self.acceleration(), bins = 50)
        plt.hist(wallPointsAccel, bins = 50, color='y')
        plt.title('Acceleration')
        plt.ylabel('Count')
        plt.xlabel('Pixel/Frame^2')

        ang = plt.figure()
        ang.add_subplot(1, 2, 1)
        plt.plot(self.angularVel()[0:MAXPLOTPOINTS])
        plt.title('Angular Velocity')
        plt.ylabel('Radian/Frame')
        plt.xlabel('Frame')
        
        ang.add_subplot(1, 2, 2)
        plt.hist(self.angularVel(), bins = 50)
        plt.hist(wallPointsAngVel, bins = 50, color='y')
        plt.title('Angular Velocity')
        plt.ylabel('Count')
        plt.xlabel('Radian/Frame')

        angAccel = plt.figure()
        angAccel.add_subplot(1, 2, 1)
        plt.plot(self.angularAccel()[0:MAXPLOTPOINTS])
        plt.title('Angular Acceleration')
        plt.ylabel('Radian/Frame^2')
        plt.xlabel('Frame')
        
        angAccel.add_subplot(1, 2, 2)
        plt.hist(self.angularAccel(), bins = 50)
        plt.hist(wallPointsAngAccel, bins = 50, color='y')
        plt.title('Angular Acceleration')
        plt.ylabel('Count')
        plt.xlabel('Radian/Frame^2')
        
    # update function for animation
    def update_quiver(self, i, data, axes):
        pos, U, V = data
        axes.set_offsets(pos[0:i+1])
        axes.set_UVC(U[0:i+1], V[0:i+1])
        return axes,
        
    # animate the motion of the hexbug
    def animate(self):     
        X = []
        Y = []
        U = []
        V = []
        pos = []
        
        for point in self.points:
            U.append(cos(point.head))
            V.append(sin(point.head))
            X.append(point.pos[0])
            Y.append(point.pos[1])
            pos.append(point.pos)
      
        fig,ax = plt.subplots(1,1)
        ax.set_xlim(min(X), max(X))
        ax.set_ylim(min(Y), max(Y))
        
        quiver = ax.quiver(X[0], Y[0], U[0], V[0], color = 'r', width = 0.003)
        data = [pos, U, V]     
        animation.FuncAnimation(fig, self.update_quiver, fargs=(data, quiver), interval=100)
        plt.show()    

# Extended Kalman Filter
# z - measurements
# x - state
# P - state covariance
# f - movement function 
# h - measurement function 
# F - movement function Jacobian
# H - measurement function Jacobian
# Q - movement covariance
# R - measurement covariance   
def EKF(z, x, P, f, h, F, H, Q, R):

    # measurement update
    I = np.eye(len(x))
    y = z - h(x)
    S = H.dot(P).dot(H.T) + R
    K = P.dot(H.T).dot(np.linalg.inv(S))
    x = x + K.dot(y)
    P = I - K.dot(H).dot(P)

    # prediction
    x = f(x)  
    P = F.dot(P).dot(F.T) + Q   
     
    return x,P
    
# state update function for a constant velocity motion model
def movementFunction(state): 
    x, y, theta, v, thetaDot = state
    
    x = x + v * cos(theta + thetaDot)
    y = y + v * sin(theta + thetaDot)
    theta = theta + thetaDot
    
    return np.array([x, y, theta, v, thetaDot])

# observation function simply returns the state
def measurementFunction(state):
    return state

# define the motion and measurement models and execute one EKF step to estimate state    
def estimateNextPosEKF(measurements, state, stateCovariance):
    
    # initialize the model    
    x, y, t, v, td = state
    movementFunctionJacobian = np.array([[1, 0, -v*sin(t+td), cos(t+td), -v*sin(t+td)],
                                         [0, 1,  v*cos(t+td), sin(t+td),  v*cos(t+td)],
                                         [0, 0, 1 + td, 0, 1 + t],
                                         [0, 0, 0, 1, 0],
                                         [0, 0, 0, 0, 1]])
    measurementFunctionJacobian = np.eye(5, 5)
    movementCovariance = np.array([[3., 0, 0, 0, 0],
                                   [0, 3., 0, 0, 0],
                                   [0, 0, .7, 0, 0],
                                   [0, 0, 0, 3., 0],
                                   [0, 0, 0, 0, .7]])
    measurementCovariance = np.eye(5, 5)
    
    # execute the EKF        
    nextState, nextCovariance = EKF(measurements,
                                    state,
                                    stateCovariance,
                                    movementFunction, 
                                    measurementFunction, 
                                    movementFunctionJacobian,
                                    measurementFunctionJacobian,
                                    movementCovariance,
                                    measurementCovariance)
    return nextState, nextCovariance

# prediction of the next state (without measurement) including a wall model
def estimateNextPos(state, wall):
    
    # hexbug bounces off walls with approximately opposite angle of incidence
    if state[0] < wall[0] or state[0] > wall[1]:
        state[2] = pi - state[2]
    if state[1] < wall[2] or state[1] > wall[3]:
        state[2] = -state[2]
        
    # execute the motion model
    return movementFunction(state) 
    
#
def tracker(testData, startPrediction, numberOfPredictions):
    
    # bound inputs   
    if startPrediction > len(testData):
        startPrediction = len(testData)
    
    # initialize
    covariance = np.eye(5,5)
    predictionArray = []
    measurementArray = []
    error = 0

    # iterate through data set    
    for i in range(0, startPrediction + numberOfPredictions):    
        
        # the measurements come directly from the data      
        if i < len(testData):        
            measurement = np.array([testData[i][0], 
                                    testData[i][1], 
                                    testData[i].head,
                                    testData[i].vel,
                                    testData[i].angVel])
            if i == 0:
                prediction = measurement # initialize prediction with first measurement
        
        # use the EKF until we start forward prediction        
        if i < startPrediction: 
            prediction, covariance = estimateNextPosEKF(measurement,
                                                        prediction, 
                                                        covariance)
        
        # to predict, use the last EKF state and motion model                                               
        else:
            
            # since angular velocity is noisy, limit its bounds           
            if prediction[4] < -MAXANGVEL:
                prediction[4] = -MAXANGVEL
            if prediction[4] > MAXANGVEL:
                prediction[4] = MAXANGVEL
            
            # motion model
            prediction = estimateNextPos(prediction, testData.wall)
            error = error + (measurement[0] - prediction[0])**2 + \
                              (measurement[1] - prediction[1])**2
        
        measurementArray.append(measurement)
        predictionArray.append(prediction)
    
    error = sqrt(error)
    return error, np.array(measurementArray), np.array(predictionArray)

# print result to screen and file
def outputResult(predictions):
    print predictions
    np.savetxt("predictions.csv", predictions, fmt = "%3.1f", delimiter=", ")

# main application logic
def main(argv):
    
    # input argument checking and read in input file
    if len(argv) < 2:
        print "Usage: {0} <centrod_file_name>".format(argv[0])
        return 1
    try:    
        testData = data()
        testData.readFile(argv[1])
    except:
        print "Unable to read data file: {0}".format(argv[1])        
        return 1 

    # filter the raw data and calculate state variables    
    testData.interpolate()
    testData.heading()    
    testData.velocity()    
    testData.angularVel()
    testData.walls()

    # loop through data NUMRUNS times making predictions and calculating error
    sumOfError = 0
    for startPrediction in range(len(testData), 0, -len(testData) / NUMRUNS):  
        
        # make the prediction
        error, measurements, predictions = tracker(testData,
                                                   startPrediction,
                                                   NUMPREDICTIONS)
        
        # plot the result
        plt.figure()
        plt.gca().invert_yaxis()
        plotStart = 0
        if startPrediction > MAXPLOTPOINTS:
            plotStart = startPrediction - MAXPLOTPOINTS
        plt.scatter(measurements.T[0][plotStart:], 
                    measurements.T[1][plotStart:], 
                    color = "b")
        plt.scatter(predictions.T[0][plotStart:], 
                    predictions.T[1][plotStart:], 
                    color = "r")
        plt.scatter(measurements[startPrediction][0],
                    measurements[startPrediction][1],
                    color = "g",
                    s = 100,
                    marker=(5,1))
    
        if NUMRUNS > 1:       
            sumOfError = sumOfError + error        
            print "L^2 error:", error
    
    if NUMRUNS > 1:    
        print "sum of L^2 error:", sumOfError
    else:
        outputResult(predictions[startPrediction:, 0:2])
    return 0

if __name__ == "__main__":
    main(sys.argv)
    #main([sys.argv[0], 'testing_video-centroid_data'])
