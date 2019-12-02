"""
Created on Fri Oct  4 00:01:19 2019

@author: christian
"""

import matplotlib.pyplot as plt
import numpy as np
import oyaml as yaml

class arc:
    def __init__(self,x0,y0,radius,theta_s,theta_f,pointsPerArcLength):
        self.x0 = x0;
        self.y0 = y0;
        self.radius = radius;
        self.theta_s = theta_s;
        self.theta_f = theta_f;
        self.pointsPerArcLength = pointsPerArcLength;
        self.length = abs(theta_f-theta_s)*radius;
        
class line:
    def __init__(self,x0,y0,x1,y1,pointsPerArcLength):
        self.x0 = x0;
        self.y0 = y0;
        self.x1 = x1;
        self.y1 = y1;
        self.pointsPerArcLength = pointsPerArcLength;
        self.length = np.sqrt((y1-y0)**2 + (x1-x0)**2);
        
class trackGenerator:
    def __init__(self):
        self.lastSegmentType = "line";
        self.chainOfSegments  = np.array([]);
        self.xCoords = []
        self.yCoords = []
        self.xRate = []
        self.yRate = []
        self.tangentAngle = []
        self.arcLength = []
        
    def __len__(self):
        print(self.arcLength[-1])
        return 666
        
    def addLine(self,x0,y0,x1,y1,numOfPoints):
        self.chainOfSegments = np.append(self.chainOfSegments,line(x0,y0,x1,y1,numOfPoints));
        
    def addArc(self,x0,y0,radius,theta_s,theta_f,numOfPoints):
        self.chainOfSegments = np.append(self.chainOfSegments,arc(x0,y0,radius,theta_s,theta_f,numOfPoints));
        
    def populatePointsAndArcLength(self):
        for segment in self.chainOfSegments:
            numOfPoints = int(segment.pointsPerArcLength*segment.length);
            #print(numOfPoints)
            if type(segment) is line:
                for i in range(0,numOfPoints):
                    self.xCoords.append(segment.x0+(segment.x1-segment.x0)/numOfPoints*i)
                    self.yCoords.append(segment.y0+(segment.y1-segment.y0)/numOfPoints*i)
                    self.xRate.append((segment.x1-segment.x0)/numOfPoints)
                    self.yRate.append((segment.y1-segment.y0)/numOfPoints)
                    if self.arcLength == []:
                        self.arcLength.append(0)
                    else:
                        self.arcLength.append(segment.length/numOfPoints+self.arcLength[-1])
            if type(segment) is arc:
                for i in range(0,numOfPoints):    
                    self.xCoords.append(segment.x0+segment.radius*np.cos((segment.theta_f - segment.theta_s)/numOfPoints*i+segment.theta_s))
                    self.yCoords.append(segment.y0+segment.radius*np.sin((segment.theta_f - segment.theta_s)/numOfPoints*i+segment.theta_s))
                    self.xRate.append(-np.sin((segment.theta_f - segment.theta_s)/numOfPoints*i+segment.theta_s))
                    self.yRate.append(np.cos((segment.theta_f - segment.theta_s)/numOfPoints*i+segment.theta_s))
                    if self.arcLength == []:
                        self.arcLength.append(0)
                    else:
                        self.arcLength.append(segment.length/numOfPoints+self.arcLength[-1])
        self.xCoords = np.array(self.xCoords)
        self.xRate = np.array(self.xRate)
        self.yCoords = np.array(self.yCoords)
        self.yRate = np.array(self.yRate)
        self.arcLength = np.array(self.arcLength)
        self.tangentAngle = np.arctan2(self.yRate,self.xRate)
        norm_rate = np.sqrt(self.xRate*self.xRate+self.yRate*self.yRate)
        self.xRate = self.xRate / norm_rate
        self.yRate = self.yRate / norm_rate
        
        self.xCoords = np.append(self.xCoords,self.xCoords)
        self.yCoords = np.append(self.yCoords,self.yCoords)
        self.xRate = np.append(self.xRate,self.xRate)
        self.yRate = np.append(self.yRate,self.yRate)
        self.arcLength = np.append(self.arcLength, self.arcLength + self.arcLength[-1] + self.arcLength[1])
        self.tangentAngle = np.append(self.tangentAngle,self.tangentAngle)
        
    def writePointsToYaml(self,path):
        temp_dict = {
                "xCoords" : self.xCoords.tolist(),
                "yCoords" : self.yCoords.tolist(),
                "xRate" : self.xRate.tolist(),
                "yRate" : self.yRate.tolist(),
                "tangentAngle" : self.tangentAngle.tolist(),
                "arcLength" : self.arcLength.tolist(),
                }
        with open(path, 'w') as outfile:
            yaml.dump(temp_dict, outfile, default_flow_style=False)
            
    def centerTrack(self):
        mean_x = (np.max(self.xCoords)+np.min(self.xCoords))/2.0
        mean_y = (np.max(self.yCoords)+np.min(self.yCoords))/2.0
        self.xCoords = self.xCoords - mean_x
        self.yCoords = self.yCoords - mean_y
                           
    def pointAtArcLength(self,arcLength):
        temp = self.arcLength - arcLength;
        idx = np.where(temp>0)
        plt.plot(self.xCoords[idx[0][0]],self.yCoords[idx[0][0]],'ro',label = 'Point at arc length ' + str(arcLength))
        plt.legend()
        
    def plotPoints(self):
        plt.plot(self.xCoords,self.yCoords,'o', label='Track')
        mean_x = (np.max(self.xCoords)+np.min(self.xCoords))/2.0
        mean_y = (np.max(self.yCoords)+np.min(self.yCoords))/2.0
        plt.plot(mean_x,mean_y,'go',label='Center')
        plt.legend()
        plt.xlabel("Position x [m]")
        plt.ylabel("Position y [m]")
        
    def plotDir(self):
        plt.plot(self.xCoords+self.xRate,self.yCoords + self.yRate,'o',label='Rate Direction')
        plt.show()
        
        
        
def main():    
    gen = trackGenerator()
    # addLine(x0, y0, x1, y1, pointsPerArcLength)
    # addArc(x0, y0, radius, theta_0, theta_1, pointsPerArcLength)

    # left turn
    radius_left = 0.075+0.205+0.025+0.1025
    gen.addLine(0, -0.125, 0, 0, 125)
    gen.addArc(-radius_left, 0, radius_left, 0, np.pi/2, round(1000*radius_left*np.pi/2))
    gen.addLine(-radius_left, radius_left, -0.125-radius_left, radius_left, 125)

    # right turn
    '''radius_right = 0.1025 + 0.075
    gen.addLine(0, -0.125, 0, 0, 125)
    gen.addArc(radius_right, 0, radius_right, np.pi, np.pi/2, round(1000*radius_right*np.pi/2))
    gen.addLine(radius_right, radius_right, 0.125 + radius_right, radius_right, 125)'''

    # go straight
    '''gen.addLine(0, -0.125, 0, 0.485 + 0.125, 485+250)'''
    
    gen.populatePointsAndArcLength()
    # gen.centerTrack()
    
    gen.plotPoints()
    gen.plotDir()
    gen.pointAtArcLength(0)
    gen.writePointsToYaml('tracks/left_turn.yaml')
    
    
if __name__ == "__main__":
   main()
      

        
