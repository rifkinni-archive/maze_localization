### !/usr/bin/env python

""" This ROS node uses proportional control to guide a robot to a specified
    distance from the obstacle immediately in front of it """

import math
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class MazeProjector(object):
    """ A ROS node that projects a virtual maze and publishes it as a topic """
    def __init__(self):
        """ Initialize a node with the specified target distance
            from the forward obstacle """

        self.scan = []
        self.odom =[]
        self.projected = [0]*360

        self.current = 0
        self.wallDistance = 1
        
    def projectMaze(self, currentNode, neighbors, direct):
        """ set projected to the virtual maze """
        
        wall = [0,0,0,0] 
        for i in neighbors:
            current = currentNode
            next = i

            if next[0] == current[0]:
                orient = 0 if next[1] > current[1] else 2
            else:
                orient = 1 if next[0] > current[0] else 3
            
            wall[orient] = 1

        wall = wall[direct:]+ wall[:direct]
        
        for i in range(0, 360):
            if math.cos(i * math.pi / 180) == 0:
                self.projected[i] = 10 #a large number
            else:
                self.projected[i] = wallDistance / math.cos(i * math.pi / 180)
        wall = [0,0,0,0]


    def detectHuman(self):
        """ look at the robot's scan and detect where the human is """

        # find a cluster of points within certain
        for i in range(0, 360):
            pass
