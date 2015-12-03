#!/usr/bin/env python

""" This ROS node uses proportional control to guide a robot to a specified
    distance from the obstacle immediately in front of it """

import math
import rospy
from sensor_msgs.msg import LaserScan
from maze_projector import MazeProjector

class MazeRobot(object):
    """ Main controller for the robot maze solver """
    def __init__(self):
        """ Main controller """
        rospy.init_node('maze_robot')

        self.scan = []
        self.projected = []

        self.MazeProjector = MazeProjector()

        self.maze = self.MazeProjector.projected

        self.pub = rospy.Publisher('/maze_scan', LaserScan, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.callbackOdom)
        rospy.Subscriber('/scan', LaserScan, self.callbackScan)
        
    def callbackScan(self, data):
        self.MazeProjector.callbackScan(data)

    def callbackOdom(self, data):
        self.MazeProjector.callbackOdom(data)

            
    def run(self):
        """ Our main 5Hz run loop """

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.pub.publish(self.MazeProjector.projectMaze())
            r.sleep()


if __name__ == '__main__':
    node = MazeRobot()

    node.run()