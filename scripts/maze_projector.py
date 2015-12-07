#!/usr/bin/env python

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

        rospy.init_node('project_maze')
        

        self.scan = []
        self.odom =[]
        self.projected = []

        self.current = 0
        
    def callbackScan(self, data):
        self.scan = data.ranges

    def callbackOdom(self, data):
        self.odom = data.pose
        
    def projectMaze(self, currentNode, neighbors):
        """ set projected to the virtual maze """
        
        wallDistance = 1

        currentNode
        for i in neighbors:
            current = currentNode
            next = i

            if next[0] == current[0]:
                orient = 0 if next[1] > current[1] else 2        
            else:
                orient = 1 if next[0] > current[0] else 3
        
        for i in range(0, 360):
            if i <= 90:
                self.projected[i] = wallDistance / math.sin(i*math.pi / 180)
            elif i <= 180:
                self.projected[i] = wallDistance / math.sin((180-i)*math.pi / 180)
            elif i <= 270:
                self.projected[i] = wallDistance / math.sin((i - 180)*math.pi / 180)
            else
                self.projected[i] = wallDistance / math.sin((360 - i)*math.pi / 180)




        return self.projected

    def detectHuman(self):
        """ look at the robot's scan and detect where the human is """

        # find a cluster of points within certain
        for i in range(0, 360):
            pass
