### !/usr/bin/env python

""" This ROS node uses proportional control to guide a robot to a specified
    distance from the obstacle immediately in front of it """

import math
# import rospy
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry

class MazeProjector(object):
    """ A ROS node that projects a virtual maze and publishes it as a topic """
    def __init__(self):
        """ Initialize a node with the specified target distance
            from the forward obstacle """

        self.scan = []
        # self.odom =[]
        self.projected = [0]*360

        # self.current = 0
        # self.wallDistance = 1

        # self.maxDistance = 5


        self.angle_centroid= 0
        self.dist_centroid = 0
        self.centroid = []
        # self.sub=rospy.Subscriber("/scan",LaserScan, self.detectHuman)
        
    def projectMaze(self, currentNode, neighbors, direct):
        """ set projected to the virtual maze """
        wallDistance = 1
        maxDistance = 5
        
        wall = [None, None, None, None] 
        for _next in neighbors:
            # # current = currentNode
            # next = i

            if _next[0] == currentNode[0]:
                orient = 0 if _next[1] > currentNode[1] else 2
            else:
                orient = 1 if _next[0] > currentNode[0] else 3
            
            wall[orient] = 1
        print wall

        wall = wall[direct:]+ wall[:direct]
        print wall

        for i in range(1, 360):
            if i <= 45 or i > 315:
                if not wall[0]:
                    self.projected[i] = wallDistance / math.cos(i*math.pi / 180)
                else:
                    c = 1.0 if i <= 45 else -1.0
                    distance = wallDistance / math.sin(i*math.pi / 180) *c
                    self.projected[i] = 0 if distance > maxDistance else distance

            elif i <= 135:
                if not wall[1]:
                    self.projected[i] = wallDistance / math.sin(i*math.pi / 180)
                else:
                    c = 1.0 if i <= 90 else -1.0
                    distance = wallDistance /  math.cos(i*math.pi / 180) * c
                    self.projected[i] = 0 if distance > maxDistance else distance

            elif i <= 225:
                if not wall[2]:
                    self.projected[i] = wallDistance / -math.cos(i*math.pi / 180)
                else:
                    c = 1.0 if i <= 180 else -1.0
                    distance = wallDistance / math.sin(i*math.pi / 180) *c
                    self.projected[i] = 0 if distance > maxDistance else distance

            elif i <= 315:
                if not wall[3]:
                    self.projected[i] = wallDistance / -math.sin( i*math.pi / 180)
                else:
                    c = -1.0 if i <= 270 else 1.0
                    distance = wallDistance / math.cos(i*math.pi / 180)*c
                    self.projected[i] = 0 if distance > maxDistance else distance
                    
        wall = [None, None, None, None] 

    def detectHuman(self):
        """ look at the robot's scan and detect where the centroid of the human is """

        # find a cluster of points within certain
        objectdict = {}
        sumx = 0
        sumy = 0
        for i in range(0, 360):
            if self.scan[i] !=0 and self.scan[i] <.5:
                locX = self.scan[i]*math.cos(i*math.pi/180.0)
                locY = self.scan[i]*math.sin(i*math.pi/180.0)
                objectdict[i] = [locX, locY]
                sumx += locX
                sumy += locY
        if len(objectdict) !=0:
            self.centroid = [sumx/len(objectdict), sumy/len(objectdict)]
            self.dist_centroid = math.sqrt(self.centroid[0]**2 + self.centroid[1]**2)
            self.angle_centroid = math.atan(self.centroid[1]/centroid[0])


if __name__ == '__main__':
    m = MazeProjector()
    current = (0, 0)
    neighbors = [(1, 0), (0, 1)]
    direct = 1
    m.projectMaze(current, neighbors, direct)