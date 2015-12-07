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
from tf.transformations import euler_from_quaternion

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

        self.graph.printGraph(start, goal)

        a = Astar(self.graph.graph, start, goal)
        self.path = a.getPath

        self.instruction = a.getInstruction()

        self.currentI = 0
        self.twist = Twist()
        self.turn = True

        self.odom = []
        self.prevOdom = []

        self.maze = self.MazeProjector.projected

        self.pubScan = rospy.Publisher('/maze_scan', LaserScan, queue_size=10)
        self.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.callbackOdom)
        rospy.Subscriber('/scan', LaserScan, self.callbackScan)
        
    def callbackScan(self, data):
        self.MazeProjector.callbackScan(data)
        self.performInstruction()

    def callbackOdom(self, data):
        self.MazeProjector.callbackOdom(data)
        self.odom = convert_pose_to_xy_and_theta(data.pose)

    def performInstruction(self):
        instruction = self.instruction[currentI]

        self.prevOdom = self.odom

        diffA = turnToAngle(instruction[0]) - differenceA(self.odom, self.prevOdom)
        diffD = distance - differenceP(self.odom, self.prevOdom)

        if diffA < .01:
            self.turn = False
        
        if diffD < .01:
            if currentI >= len(self.instruction):
                self.twist.linear.x = 0
                self.twist.linear.z = 0
                print "done traversing the maze"

            currentI += 1
            self.turn = True
            self.prevOdom = self.odom

            currentNode = self.path[currentI]
            neighbors = self.graph[currentNode[0]][currentNode[1]].neighbors

            # pass in neighbors and current node to MazeProjector
            self.MazeProjector.projectMaze(currentNode, neighbors)


        if self.turn:
            self.twist.angular.z = c * diffA
        else:
            self.twist.linear.x = c*diffD

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
        return math.pow((current[0] - previous[0])**2 + (current[1] - previous[1])**2, 1/2)

    def differenceA(self, current, previous):
        return current[2] - previous.x[2]
        

    def convert_pose_to_xy_and_theta(pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        return pose.position.x, pose.position.y, angles[2]
            
    def run(self):
        """ Our main 5Hz run loop """

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.pubSub.publish(self.MazeProjector.projectMaze())
            self.pubVel.publish(self.twist)
            r.sleep()


if __name__ == '__main__':
    node = MazeRobot()

    node.run()