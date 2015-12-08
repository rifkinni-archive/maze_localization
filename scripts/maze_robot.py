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

class MazeRobot(object):
    """ Main controller for the robot maze solver """
    def __init__(self):
        """ Main controller """
        rospy.init_node('maze_robot')

<<<<<<< HEAD
        self.odom = []
        self.prevOdom = []
=======
        self.scan = []
        self.projected = []

        # self.MazeProjector = MazeProjector()
        
        self.graph = Graph(10)

        start = (0, 0)
        goal = (random.randint(0, g.size - 1), random.randint(0, g.size - 1))

        self.graph.printGraph(start, goal)
>>>>>>> b621569904a3b41de9387ef6863b41ddcd56bddf


        self.MazeProjector = MazeProjector()
        self.solver = MazeSolver()
        self.instructions = self.solver.getInstructions()

        self.currentI = 0
        self.twist = Twist()
        self.laserScan = LaserScan()
        self.turn = True


        # self.maze = self.MazeProjector.projected

        self.pubScan = rospy.Publisher('/maze_scan', LaserScan, queue_size=10)
        self.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.callbackOdom)
        rospy.Subscriber('/scan', LaserScan, self.callbackScan)
        
    def callbackScan(self, data):
        # self.MazeProjector.callbackScan(data)
        self.performInstruction()

    def callbackOdom(self, data):
<<<<<<< HEAD
        self.MazeProjector.callbackOdom(data)
        self.odom = self.convert_pose_to_xy_and_theta(data.pose)
=======
        # self.MazeProjector.callbackOdom(data)
        self.odom = convert_pose_to_xy_and_theta(data.pose)
>>>>>>> b621569904a3b41de9387ef6863b41ddcd56bddf

    def performInstruction(self):
        instruction = self.instructions[self.currentI]
        distance = 1
        c = 1

        self.prevOdom = self.odom

        diffA = self.turnToAngle(instruction[0]) - self.differenceA(self.odom, self.prevOdom)
        diffD = distance - self.differenceP(self.odom, self.prevOdom)

        if diffA < .01:
            self.turn = False
        
        if diffD < .01:
            if self.currentI >= len(self.instructions):
                self.twist.linear.x = 0
                self.twist.linear.z = 0
                print "done traversing the maze"

            self.currentI += 1
            self.turn = True
            self.prevOdom = self.odom

<<<<<<< HEAD
            currentNode = self.solver.path[self.currentI]
            neighbors = self.solver.getNeighbors(currentNode)

            # pass in neighbors and current node to MazeProjector
            self.MazeProjector.projectMaze(currentNode, neighbors)
            self.laserScan.ranges = self.MazeProjector.projected
=======
            # currentNode = self.path[currentI]
            # neighbors = self.graph[currentNode[0]][currentNode[1]].neighbors
>>>>>>> b621569904a3b41de9387ef6863b41ddcd56bddf

            # # pass in neighbors and current node to MazeProjector
            # self.MazeProjector.projectMaze(currentNode, neighbors, self.instruction[currentI][1])

        # c is a constant that doesnt exist yet
        if self.turn:
            self.twist.angular.z = c * diffA
        else:
            self.twist.linear.x = c *diffD

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
        if len(current) and len(previous):
            return math.pow((current[0] - previous[0])**2 + (current[1] - previous[1])**2, 1/2)
        else:
            return 0

    def differenceA(self, current, previous):
        if len(current) and len(previous):
            print current, previous
            return current[2] - previous[2]
        else:
            return 0
        

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
<<<<<<< HEAD
            self.pubScan.publish(self.laserScan)
=======
            # self.pubSub.publish("?")
>>>>>>> b621569904a3b41de9387ef6863b41ddcd56bddf
            self.pubVel.publish(self.twist)
            r.sleep()


if __name__ == '__main__':
    node = MazeRobot()

    node.run()