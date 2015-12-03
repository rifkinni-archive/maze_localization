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
        self.projected = []

        self.current = 0
        
    def callbackScan(self, data):
        self.scan = data.ranges


    def callbackOdom(self, data):
        self.scan = data.pose
        
    def projectMaze(self):
        """ set projected to the virtual maze """
        self.projected = self.scan
        
        distance = 1

        # somehow make it depended on the odom also

        for i in range(0, 360):
            if i >= 90:
                self.projected[i] = distance / math.sin(i*math.pi / 180)
            elif i >= 180:
                self.projected[i] = distance / math.sin(i*math.pi / 180)
            elif i >= 270:
                self.projected[i] = distance / math.sin(i*math.pi / 180)
            else
                self.projected[i] = distance / math.sin(i*math.pi / 180)




        return self.projected

    def detectHuman(self):
        """ look at the robot's scan and detect where the human is """


