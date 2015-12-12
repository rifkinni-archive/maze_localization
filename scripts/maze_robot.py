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

class MazeRobot(object):
    """ Main controller for the robot maze solver """
    def __init__(self):
        """ Main controller """
        rospy.init_node('maze_robot')

        self.odom = None
        self.prevOdom = None

        self.projector = MazeProjector()
        self.solver = MazeSolver()

        self.currentI = 0
        self.twist = Twist()
        self.laserScan = LaserScan()
        self.turn = True

        self.pubScan = rospy.Publisher('/maze_scan', LaserScan, queue_size=10)
        self.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.callbackOdom)
        rospy.Subscriber('/scan', LaserScan, self.callbackScan)

        
    def callbackScan(self, data):
        """ updates on new scan data
            data: LaserScan data
        """
        self.projector.scan = data.ranges
        # self.performInstruction()

    def callbackOdom(self, data):
        self.projector.odom = data.pose
        self.odom = self.convert_pose_to_xy_and_theta(data.pose)
        if not self.prevOdom:
            self.prevOdom = self.odom

    def performInstruction(self):
        """ acts as a run loop to perform instructions

        """
        if self.currentI >= len(self.solver.instructions):
            print "here!", self.currentI, len(self.solver.instructions)
            return

        c = .5
        diffPos, diffAng = self.calcDifference(self.odom, self.prevOdom)
        # diffAng = self.turnToAngle(instruction[0]) - self.differenceAng(self.odom, self.prevOdom)
        # diffAng = instruction[0] - self.differenceAng(self.odom, self.prevOdom)
        # diffPos = distance - self.differencePos(self.odom, self.prevOdom)

        if abs(diffAng) < .05 or abs(diffAng) > 2*math.pi - .5:
            self.turn = False
        
        if abs(diffPos) < .05:

            self.currentI += 1
            self.turn = True
            self.prevOdom = self.odom

            currentNode = self.solver.path[self.currentI]
            # neighbors = self.solver.getNeighbors(currentNode)

            # get projected maze
            self.laserScan.ranges = self.projector.projected
        
        if self.turn:
            self.twist.angular.z = c * diffAng
            self.twist.linear.x = 0
        else:
            self.twist.linear.x = c *diffPos
            self.twist.angular.z = 0


    def calcDifference(self, current, previous):
        """ calculate the difference in position and orientation between current and previous odometry
            current: current odometry
            previous: previous odometry
            returns tuple of form (difference in position, difference in orientation)
        """
        instruction = self.solver.instructions[self.currentI]
        distance = 1

        if current and previous:
            diffPos = distance - math.sqrt((current[0] - previous[0])**2 + (current[1] - previous[1])**2)
            diffAng = instruction[0] - (current[2] - previous[2])
            return diffPos, diffAng
        else:
            return 0, 0
        

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
        """ Our main 5Hz run loop """

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.odom:
                if self.currentI < len(self.solver.instructions):
                    self.performInstruction()
                    self.pubScan.publish(self.laserScan)
                    self.pubVel.publish(self.twist)
                    r.sleep()

                else:
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0
                    print "done traversing the maze"
                    self.pubVel.publish(self.twist)
                    break 
                    


if __name__ == '__main__':
    node = MazeRobot()

    node.run()