#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import matplotlib.pyplot as plt
import numpy as np
import oyaml as yaml

##Houses information about a particular arc on a track
class arc:
    def __init__(self,x0,y0,radius,theta_s,theta_f,pointsPerArcLength):
        self.x0 = x0
        self.y0 = y0
        self.radius = radius
        self.theta_s = theta_s
        self.theta_f = theta_f
        self.pointsPerArcLength = pointsPerArcLength
        self.length = abs(theta_f-theta_s)*radius

##Houses information about a particular straight line on a track
class line:
    def __init__(self,x0,y0,x1,y1,pointsPerArcLength):
        self.x0 = x0
        self.y0 = y0
        self.x1 = x1
        self.y1 = y1
        self.pointsPerArcLength = pointsPerArcLength
        self.length = np.sqrt((y1-y0)**2 + (x1-x0)**2)

##Main class which can be used to generate the track
class trackGenerator:
    ##The constructor
    def __init__(self):
        self.lastSegmentType = "line"
        self.chainOfSegments  = np.array([])
        self.xCoords = []
        self.yCoords = []
        self.xRate = []
        self.yRate = []
        self.tangentAngle = []
        self.curvature = []

    ##Chains a line to the list of segments
    def addLine(self,x0,y0,x1,y1,numOfPoints):
        self.chainOfSegments = np.append(self.chainOfSegments,line(x0,y0,x1,y1,numOfPoints))

    ##Chains an arc to the list of segments
    def addArc(self,x0,y0,radius,theta_s,theta_f,numOfPoints):
        self.chainOfSegments = np.append(self.chainOfSegments,arc(x0,y0,radius,theta_s,theta_f,numOfPoints))

    ##After the track has been constructed points on the track can be generated using this method
    def populatePointsAndArcLength(self):
        for segment in self.chainOfSegments:
            numOfPoints = int(segment.pointsPerArcLength*segment.length)
            if type(segment) is line:
                for i in range(0,numOfPoints):
                    self.xCoords.append(segment.x0+(segment.x1-segment.x0)/float(numOfPoints)*i)
                    self.yCoords.append(segment.y0+(segment.y1-segment.y0)/float(numOfPoints)*i)
                    self.curvature.append(0.0)
            if type(segment) is arc:
                for i in range(0,numOfPoints):
                    self.xCoords.append(segment.x0+segment.radius*np.cos((segment.theta_f - segment.theta_s)/float(numOfPoints)*i+segment.theta_s))
                    self.yCoords.append(segment.y0+segment.radius*np.sin((segment.theta_f - segment.theta_s)/float(numOfPoints)*i+segment.theta_s))
                    curv = 1/float(segment.radius)
                    if segment.theta_s > segment.theta_f:
                        curv = -curv
                    self.curvature.append(curv)

        self.xCoords = np.array(self.xCoords)
        self.yCoords = np.array(self.yCoords)
        self.xRate = np.diff(self.xCoords)
        self.yRate = np.diff(self.yCoords)
        self.xCoords = self.xCoords[:-1]
        self.yCoords = self.yCoords[:-1]
        self.tangentAngle = np.arctan2(self.yRate,self.xRate)
        norm_rate = np.sqrt(self.xRate*self.xRate+self.yRate*self.yRate)
        self.xRate = self.xRate / norm_rate
        self.yRate = self.yRate / norm_rate
        self.curvature = np.array(self.curvature)

    ##Writes the track to a yaml file, specified by path
    def writePointsToYaml(self,path):
        temp_dict = {
                "xCoords" : self.xCoords.tolist(),
                "yCoords" : self.yCoords.tolist(),
                "tangentAngle" : self.tangentAngle.tolist(),
                "curvature" : self.curvature.tolist(),
                }
        with open(path, 'w') as outfile:
            yaml.dump(temp_dict, outfile, default_flow_style=False)

    ##Centers the track using infinity norms
    def centerTrack(self):
        mean_x = (np.max(self.xCoords)+np.min(self.xCoords))/2.0
        mean_y = (np.max(self.yCoords)+np.min(self.yCoords))/2.0
        self.xCoords = self.xCoords - mean_x
        self.yCoords = self.yCoords - mean_y

    ##Plots the track
    def plotPoints(self):
        print(self.yCoords.shape)
        print(self.tangentAngle.shape)
        plt.plot(self.xCoords,self.yCoords,'.', label='Track')
        mean_x = (np.max(self.xCoords)+np.min(self.xCoords))/2.0
        mean_y = (np.max(self.yCoords)+np.min(self.yCoords))/2.0
        plt.plot(mean_x,mean_y,'go',label='Center')
        plt.legend()
        plt.xlabel("Position x [m]")
        plt.ylabel("Position y [m]")

        axis_min = min(self.xCoords.min(), self.yCoords.min())
        axis_max = max(self.xCoords.max(), self.yCoords.max())
        plt.xlim(axis_min, axis_max)
        plt.ylim(axis_min, axis_max)
        plt.show()

    ##Plots the direction of rate of change of each point on the track
    def plotDir(self):
        plt.plot(self.xRate,self.yRate,'.',label='Rate Direction')
        #plt.plot(np.array(range(self.tangentAngle.shape[0])),self.tangentAngle,'.')
        plt.show()


def main():
    '''
    Equal spacing: currently every point is 2 mm apart
    '''
    gen = trackGenerator()
    # addLine(x0, y0, x1, y1, pointsPerArcLength)
    # addArc(x0, y0, radius, theta_0, theta_1, pointsPerArcLength)

    # left turn
    '''radius_left = 0.205 + 0.1025 + 0.024 + 0.024
    gen.addLine(0, -0.30, 0, 0, 200)
    gen.addArc(-radius_left, 0, radius_left, 0, np.pi / 2, 200)
    gen.addLine(-radius_left, radius_left, -0.10 - radius_left, radius_left, 200)'''

    # left turn 2
    radius_left2 = 0.205 + 0.1025 + 0.024 + 0.024 + 0.05
    gen.addLine(0, -0.30, 0, -0.05, 200)
    gen.addArc(-radius_left2, -0.05, radius_left2, 0, np.pi / 2, 200)
    gen.addLine(-radius_left2, radius_left2-0.05, -0.05 - radius_left2, radius_left2-0.05, 200)


    '''
    # right turn
    radius_right = 0.1025 + 0.024
    gen.addLine(0, -0.30, 0, 0, 200)
    gen.addArc(radius_right, 0, radius_right, np.pi, np.pi/2, 200)
    gen.addLine(radius_right, radius_right, 0.10 + radius_right, radius_right, 200)

    # right turn 2 --> larger angle
    radius_right1 = radius_right + 0.1
    gen.addLine(0, -0.30, 0, -0.1, 200)
    gen.addArc(radius_right1, -0.1, radius_right1, np.pi, np.pi/2, 200)

    # right turn 2 --> largest angle
    radius_right2 = 0.1025/(1-1/np.sqrt(2))
    gen.addLine(0, -0.30, 0, -radius_right2/np.sqrt(2)-0.024, 200)
    gen.addArc(radius_right2, -radius_right2/np.sqrt(2)-0.024, radius_right2, np.pi, 1.91, 200)

    # right turn 3 --> translated
    gen.addLine(0, -0.30, 0, -0.1, 200)
    gen.addArc(radius_right, -0.1, radius_right, np.pi, np.pi / 2, 200)
    gen.addLine(radius_right, -0.1 + radius_right, 0.10 + radius_right, -0.1 + radius_right, 200)
    '''
    # go straight
    '''
    gen.addLine(0, -0.30, 0, 0.435 + 0.048 + 0.10, 200)
    '''

    gen.populatePointsAndArcLength()
    # gen.centerTrack()

    #gen.plotPoints()
    #gen.plotDir()
    gen.writePointsToYaml('left1.yaml')

if __name__ == "__main__":
   main()



