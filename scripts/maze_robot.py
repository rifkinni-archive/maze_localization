#!/usr/bin/env python

""" This ROS node uses proportional control to guide a robot to a specified
    distance from the obstacle immediately in front of it """

import math
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from maze_projector import MazeProjector
from geometry_msgs.msg import Twist, Vector3
from maze_solver import MazeSolver
# from maze import Graph
# from astar import Astar
from tf.transformations import euler_from_quaternion
import sys

class MazeRobot(object):
    """ Main controller for the robot maze solver """
    def __init__(self):
        """ Main controller """
        rospy.init_node('maze_robot')

        self.odom = None
        self.prevOdom = None


        self.projector = MazeProjector()
        self.solver = MazeSolver()
        self.instructions = self.solver.getInstructions()

        self.currentI = 0
        self.twist = Twist()
        self.laserScan = LaserScan()
        self.turn = True

        self.pubScan = rospy.Publisher('/maze_scan', LaserScan, queue_size=10)
        self.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.callbackOdom)
        rospy.Subscriber('/scan', LaserScan, self.callbackScan)

        
    def callbackScan(self, data):
        self.projector.scan = data.ranges
        self.performInstruction()

    def callbackOdom(self, data):
        self.projector.odom = data.pose
        self.odom = self.convert_pose_to_xy_and_theta(data.pose)
        if not self.prevOdom:
            self.prevOdom = self.odom

    def performInstruction(self):
        if self.currentI >= len(self.instructions):
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            print "done traversing the maze"
            sys.exit()

        instruction = self.instructions[self.currentI]
        distance = 1
        c = .5

        diffA = self.turnToAngle(instruction[0]) - self.differenceA(self.odom, self.prevOdom)
        diffD = distance - self.differenceP(self.odom, self.prevOdom)

        if abs(diffA) < .05:
            print "done turning"
            self.turn = False
        
        if abs(diffD) < .05:
            print "next node"

            self.currentI += 1
            self.turn = True
            self.prevOdom = self.odom

            currentNode = self.solver.path[self.currentI]
            neighbors = self.solver.getNeighbors(currentNode)

            # pass in neighbors and current node to MazeProjector
            # self.projector.projectMaze(currentNode, neighbors)
            self.laserScan.ranges = self.projector.projected

            # # pass in neighbors and current node to MazeProjector

        # c is a constant that doesnt exist yet
        if self.turn:
            self.twist.angular.z = c * diffA
            self.twist.linear.x = 0
        else:
            self.twist.linear.x = c *diffD
            self.twist.angular.z = 0

    def turnToAngle(self, instruction):
        if instruction == "left":
            # turn left
            return math.pi/2               
        elif instruction == "right":
            # turn right
            return -math.pi/2
            
        elif instruction == "no turn":
            return 0
        elif instruction == "full":
            # turn left
            # turn left
            return math.pi

    def differenceP(self, current, previous):
        if current and previous:
            return math.sqrt((current[0] - previous[0])**2 + (current[1] - previous[1])**2)
        else:
            return 0

    def differenceA(self, current, previous):
        if current and previous:
            return current[2] - previous[2]
        else:
            return 0

    def radToD(self, angle):
        return angle*180/math.pi
        

    def convert_pose_to_xy_and_theta(self, pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.pose.orientation.x,
                             pose.pose.orientation.y,
                             pose.pose.orientation.z,
                             pose.pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        return pose.pose.position.x, pose.pose.position.y, angles[2]
            
    def run(self):
        """ Our main 5Hz run loop """

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.pubScan.publish(self.laserScan)
            self.pubVel.publish(self.twist)
            r.sleep()


if __name__ == '__main__':
    node = MazeRobot()

    node.run()