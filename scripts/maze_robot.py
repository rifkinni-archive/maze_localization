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
from tf.transformations import euler_from_quaternion

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

        
    def callbackScan(self, data):
        """ updates on new scan data
            data: LaserScan data
        """
        if not self.scan:
            self.laserScan = data
        # self.laserScan.header = data.header
        self.scan = data.ranges


    def callbackOdom(self, data):
        """ updates on new odom data
            data: Odometry data
        """
        self.odom = self.convert_pose_to_xy_and_theta(data.pose)
        if not self.prevOdom: #first reading
            self.prevOdom = self.odom #no change

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
            self.currentI += 1 #increment instruction
            self.turn = True 
            self.prevOdom = self.odom #update odometry
            wall = self.getWalls(instruction[1])
            self.projected = self.projectMaze(wall) #update laser scan 
            self.laserScan.ranges = tuple(self.projectMaze(wall)) #update laser scan 
            
            self.laserScan.header.stamp = rospy.Time.now()
            self.laserScan.header.frame_id = "base_laser_link"

        if self.turn: #set angular velocity
            self.twist.angular.z = c * diffAng
            self.twist.linear.x = 0
            # self.rotateWalls(self.odom[2])

        else: #set linear velocity
            self.twist.linear.x = c *diffPos
            self.twist.angular.z = 0


    def angle_normalize(self, z):
        """ convenience function to map an angle to the range [-pi,pi] """
        return math.atan2(math.sin(z), math.cos(z))

    def angle_diff(self, a, b):
        """ Calculates the difference between angle a and angle b (both should be in radians)
            the difference is always based on the closest rotation from angle a to angle b
            examples:
                angle_diff(.1,.2) -> -.1
                angle_diff(.1, 2*math.pi - .1) -> .2
                angle_diff(.1, .2+2*math.pi) -> -.1
        """
        a = self.angle_normalize(a)
        b = self.angle_normalize(b)
        d1 = a-b
        d2 = 2*math.pi - math.fabs(d1)
        if d1 > 0:
            d2 *= -1.0
        if math.fabs(d1) < math.fabs(d2):
            return d1
        else:
            return d2

    def calcDifference(self, instruction):
        """ calculate the difference in position and orientation between current and previous odometry
            returns tuple of form (difference in position, difference in orientation)
        """
        distance = .5 #distance between nodes
        # currAng = current[2]%(2*math.pi)
        # prevAng = previous[2]%(2*math.pi)
        diffPos = distance - math.sqrt((self.odom[0] - self.prevOdom[0])**2 + (self.odom[1] - self.prevOdom[1])**2)
        diffAng = instruction[0] - self.angle_diff(self.odom[2],self.prevOdom[2])
        # if self.turn:
        #     print "angle difference", diffAng
        #     print "odom current", currAng, "odom prev", prevAng
        #     print "odom difference", current[2] - previous[2]
        #     print "-----"
        return diffPos, diffAng

    def getWalls(self, orientation):
        """ get a representation of maze walls the robot can understand
            currentNode: coordinates of current node
            orientation: current orientation of the robot
            returns: list of length 4 with binary entries
                True is a path
                False is a wall
        """
        currentNode = self.solver.path[self.currentI]
        neighbors = self.solver.getNeighbors(currentNode)
        walls = [None]*4 #default is all walls
        for nextNode in neighbors:
            nextOrient = self.solver.getNextOrientation(currentNode, nextNode)
            walls[nextOrient] = 1 #update list with True where paths exist
        return walls[orientation:] + walls[:orientation] #rotated depending on robot's position


    def projectMaze(self, wall):
        """ get 'laser scan' ranges for the virtual maze based on surrounding walls
            wall: output from getWalls
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

    def rotateWalls(self, orientation):
        if self.projected:
            orientation = int(orientation * 180/math.pi)
            rotatedProjection = self.projected[orientation:] + self.projected[:orientation]
            self.laserScan.ranges = tuple(rotatedProjection)
            # self.pubScan.publish(self.laserScan) #publish maze scan



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
                self.pubScan.publish(self.laserScan) #publish maze scan
                self.pubVel.publish(self.twist)
                r.sleep()

            else: 
                self.twist.linear.x = 0 #stop the robot
                self.twist.angular.z = 0
                print "done traversing the maze"
                self.pubVel.publish(self.twist)
                break #exit
            
if __name__ == '__main__':
    node = MazeNavigator()
    node.run()