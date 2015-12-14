### !/usr/bin/env python

""" This ROS node uses proportional control to guide a robot to a specified
    distance from the obstacle immediately in front of it """

import math
import matplotlib.pyplot as plt
import numpy as np


class Projector(object):
    """ A ROS node that projects a virtual maze and publishes it as a topic """
    def __init__(self):
        """ Initialize a node with the specified target distance
            from the forward obstacle """

        self.projected = [0]*360

        self.current = 0
        self.wallDistance = 1

        self.maxDistance = 5
        
    def projectMaze(self):
        """ set projected to the virtual maze """
        
        wall = [None, 1, 1, None]

        for i in range(1, 360):
            if i <= 45 or i > 315:
                if not wall[0]:
                    self.projected[i] = self.wallDistance / math.cos(i*math.pi / 180)
                else:
                    c = 1.0 if i <= 45 else -1.0
                    distance = self.wallDistance / math.sin(i*math.pi / 180) *c
                    self.projected[i] = 0 if distance > self.maxDistance else distance

            elif i <= 135:
                if not wall[1]:
                    self.projected[i] = self.wallDistance / math.sin(i*math.pi / 180)
                else:
                    c = 1.0 if i <= 90 else -1.0
                    distance = self.wallDistance /  math.cos(i*math.pi / 180) * c
                    self.projected[i] = 0 if distance > self.maxDistance else distance

            elif i <= 225:
                if not wall[2]:
                    self.projected[i] = self.wallDistance / -math.cos(i*math.pi / 180)
                else:
                    c = 1.0 if i <= 180 else -1.0
                    distance = self.wallDistance / math.sin(i*math.pi / 180) *c
                    self.projected[i] = 0 if distance > self.maxDistance else distance

            elif i <= 315:
                if not wall[3]:
                    self.projected[i] = self.wallDistance / -math.sin( i*math.pi / 180)
                else:
                    c = -1.0 if i <= 270 else 1.0
                    distance = self.wallDistance / math.cos(i*math.pi / 180)*c
                    self.projected[i] = 0 if distance > self.maxDistance else distance
        
        return self.projected

        # wall = [None, None, None, None] 
n = Projector()
# plt.axis([-5, 5, -5, 5])

r = np.asarray(n.projectMaze())
theta = np.arange(0,360)
theta = theta /180.0 * np.pi
# print r

ax = plt.subplot(111, projection='polar')
c = plt.scatter(theta, r)
c.set_alpha(0.75)

plt.show()
