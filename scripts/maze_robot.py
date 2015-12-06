#!/usr/bin/env python

""" This ROS node uses proportional control to guide a robot to a specified
    distance from the obstacle immediately in front of it """

import math
import rospy
from sensor_msgs.msg import LaserScan
from maze_projector import MazeProjector
from geometry_msgs.msg import Twist, Vector3
from maze import Graph
from astar import Astar

class MazeRobot(object):
    """ Main controller for the robot maze solver """
    def __init__(self):
        """ Main controller """
        rospy.init_node('maze_robot')

        self.scan = []
        self.projected = []

        self.MazeProjector = MazeProjector()
        self.graph = Graph(10)

        start = (0, 0)
        goal = (random.randint(0, g.size - 1), random.randint(0, g.size - 1))

        g.printGraph(start, goal)

        a = Astar(g.graph, start, goal)

        self.instruction = a.getInstruction()

        self.currentI = 0
        self.twist = Twist()

        self.maze = self.MazeProjector.projected

        self.pub = rospy.Publisher('/maze_scan', LaserScan, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.callbackOdom)
        rospy.Subscriber('/scan', LaserScan, self.callbackScan)
        
    def callbackScan(self, data):
        self.MazeProjector.callbackScan(data)
        self.performInstruction()

    def callbackOdom(self, data):
        self.MazeProjector.callbackOdom(data)

    def performInstruction(self):
        instruction = self.instruction[currentI]

        if instruction[0] == "left":
            # turn left
            
        elif instruction[0] == "right":
            # turn right
            
        elif instruction[0] == "no turn":
            pass
        elif instruction[0] == "full":
            # turn left
            # turn left
        else:
            print "instruction is not valid"
            
    def run(self):
        """ Our main 5Hz run loop """

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.pub.publish(self.MazeProjector.projectMaze())
            r.sleep()


if __name__ == '__main__':
    node = MazeRobot()

    node.run()