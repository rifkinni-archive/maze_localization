#!/usr/bin/env python

""" This ROS node uses proportional control to guide a robot to a specified
    distance from the obstacle immediately in front of it """

import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from maze_projector import MazeProjector
from geometry_msgs.msg import Twist, Vector3
from maze_solver import MazeSolver
from tf import TransformListener, TransformBroadcaster
from tf.transformations import euler_from_quaternion
from helpers import *

class MazeNavigator(object):
    """ Main controller for the robot maze solver """
    def __init__(self):
        """ Main controller """
        rospy.init_node('maze_navigator')

        self.odom = None
        self.prevOdom = None
        self.scan = []
        self.projected = []

        self.solver = MazeSolver()
        self.listener = TransformListener()
        self.broadcaster = TransformBroadcaster()

        self.currentI = 0 #index to keep track of our instruction
        self.twist = Twist()
        self.laserScan = LaserScan()
        self.turn = True #turning or moving straight

        #publish robot commands and fake lidar data
        self.pubScan = rospy.Publisher('/maze_scan', LaserScan, queue_size=10)
        self.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 

        #subscribe to robot position and real lidar data
        rospy.Subscriber('/odom', Odometry, self.callbackOdom)
        rospy.Subscriber('/scan', LaserScan, self.callbackScan)

        self.solver.visualize((0, 0)) 

        
    def callbackScan(self, data):
        """ updates on new scan data
            data: LaserScan data
        """
        if not self.scan:
            self.laserScan = data
        self.scan = data.ranges


    def callbackOdom(self, data):
        """ updates on new odom data
            data: Odometry data
        """
        self.odom = convert_pose_to_xy_and_theta(data.pose)
        if not self.prevOdom: #first reading
            self.prevOdom = self.odom #no change

    def updateNode(self, instruction):
        """ updates visualization and publishes new scan data when a new node is reached
            instruction: new instruction
        """
        self.currentI += 1 #increment instruction
        newNode = self.solver.path[self.currentI]
        
        self.turn = True 
        self.prevOdom = self.odom #update odometry
        
        wall = self.getWalls(instruction[1], newNode)
        self.projected = self.projectMaze(wall) #get new laser scan 
        
        stamp = rospy.Time.now()
        self.laserScan.ranges = tuple(self.projectMaze(wall)) #update laser scan
        self.laserScan.header=Header(stamp=rospy.Time.now(),frame_id="base_laser_link")
        fix_map_to_odom_transform(self, stamp, newNode, instruction[1], self.listener, self.broadcaster)
        self.solver.visualize(newNode) #update visualization



    def performInstruction(self):
        """ sets twist and updates maze scan
            based on current instruction and odometry reading
        """
        if not self.odom or not self.prevOdom:
            return

        c = .5 #proportional control constant
        instruction = self.solver.instructions[self.currentI]
        diffPos, diffAng = self.calcDifference(instruction) #difference in position, difference in angle

        if abs(diffAng)%(2*math.pi) < .05: #turned to correct orientation
            self.turn = False
        
        if abs(diffPos) < .05: #moved forward successfully to next node
            self.updateNode(instruction)

        if self.turn: #set angular velocity
            self.twist.angular.z = c * diffAng
            self.twist.linear.x = 0

        else: #set linear velocity
            self.twist.linear.x = c *diffPos
            self.twist.angular.z = 0

    def calcDifference(self, instruction):
        """ calculate the difference in position and orientation between current and previous odometry
            returns tuple of form (difference in position, difference in orientation)
        """
        distance = .5 #distance between nodes
        diffPos = distance - math.sqrt((self.odom[0] - self.prevOdom[0])**2 + (self.odom[1] - self.prevOdom[1])**2)
        diffAng = instruction[0] - angle_diff(self.odom[2],self.prevOdom[2])
        return diffPos, diffAng

    def getWalls(self, orientation, currentNode):
        """ get a representation of maze walls the robot can understand
            currentNode: coordinates of current node
            orientation: current orientation of the robot
            returns: list of length 4 with binary entries
                True is a path
                False is a wall
        """
        neighbors = self.solver.getNeighbors(currentNode)
        walls = [None]*4 #default is all walls
        for nextNode in neighbors:
            nextOrient = self.solver.getNextOrientation(currentNode, nextNode)
            walls[nextOrient] = 1 #update list with True where paths exist
        return walls[orientation:] + walls[:orientation] #rotated depending on robot's position


    def projectMaze(self, wall):
        """ get 'laser scan' ranges for the virtual maze based on surrounding walls
            wall: list with binary entries, output from getWalls
            returns a list of length 359 with maze scan data to be published
        """
        wallDistance = .2
        maxDistance = .4
        projected = [0]*361

        for i in range(1, 360):
            if i <= 45 or i > 315:
                if not wall[0]:
                   projected[i] = wallDistance / math.cos(i*math.pi / 180)
                else:
                    c = 1.0 if i <= 45 else -1.0
                    distance = wallDistance / math.sin(i*math.pi / 180) *c
                    projected[i] = 0 if distance > maxDistance else distance

            elif i <= 135:
                if not wall[3]:
                    projected[i] = wallDistance / math.sin(i*math.pi / 180)
                else:
                    c = 1.0 if i <= 90 else -1.0
                    distance = wallDistance /  math.cos(i*math.pi / 180) * c
                    projected[i] = 0 if distance > maxDistance else distance

            elif i <= 225:
                if not wall[2]:
                    projected[i] = wallDistance / -math.cos(i*math.pi / 180)
                else:
                    c = 1.0 if i <= 180 else -1.0
                    distance = wallDistance / math.sin(i*math.pi / 180) *c
                    projected[i] = 0 if distance > maxDistance else distance

            elif i <= 315:
                if not wall[1]:
                    projected[i] = wallDistance / -math.sin( i*math.pi / 180)
                else:
                    c = -1.0 if i <= 270 else 1.0
                    distance = wallDistance / math.cos(i*math.pi / 180)*c
                    projected[i] = 0 if distance > maxDistance else distance
        return projected


    def convert_pose_to_xy_and_theta(self, pose):
        """ pose: geometry_msgs.Pose object
            returns tuple of form (x, y, yaw)
        """
        orientation_tuple = (pose.pose.orientation.x,
                             pose.pose.orientation.y,
                             pose.pose.orientation.z,
                             pose.pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        return pose.pose.position.x, pose.pose.position.y, angles[2]
            
    def run(self):
        """ Our main 5Hz run loop
        """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.currentI < len(self.solver.instructions): #still have instructions to perform
                self.performInstruction()
                self.pubScan.publish(self.laserScan) #publish scans
                self.pubVel.publish(self.twist)
                r.sleep()

            else: 
                self.twist.linear.x = 0 #stop the robot
                self.twist.angular.z = 0
                self.pubVel.publish(self.twist)
                print "done traversing the maze"
                break #exit
            
if __name__ == '__main__':
    node = MazeNavigator()
    node.run()