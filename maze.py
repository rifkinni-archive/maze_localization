import numpy as np
import random
import matplotlib.pyplot as plt
from Astar2 import Astar


class Node(object):
  def __init__(self, x, y):
    # self.nodeType = nodeType
    self.coordX = x #position on a grid
    self.coordY = y 
    self.neighbors = []

class Graph(object):
  def __init__(self, size):
    self.size = size
    self.graph = [[0 for x in range(self.size)] for x in range(self.size)]
    self.stack = [(0,0)]
    self.test = []
    self.recursiveBacktracking((0, 0))

  def getNodeChoices(self, i, j):
    directions = [(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)]
    choices = []
    for x, y in directions:
      if (x >= 0) and (y >=0):
        if (x < self.size) and (y < self.size):
          if self.graph[x][y] == 0:
            choices.append((x, y))
    random.shuffle(choices)
    return choices

  def recursiveBacktracking(self, indices):
    i, j = indices
    choices = self.getNodeChoices(i, j)

    if self.graph[i][j] == 0:
      self.graph[i][j] = Node(i, j)
      
    if choices: #we are not blocked in
      choice = choices[0]
      self.stack.append((i, j))
      self.recursiveBacktracking(choice)
      
    else: #we are blocked in
      if len(self.stack):
        newI, newJ = self.stack.pop()
        self.updateNeighbors((newI, newJ), (i, j))
        self.recursiveBacktracking((newI, newJ))
      #else we have hit every node in the maze

  def updateNeighbors(self, coord1, coord2):
    node1 = self.graph[coord1[0]][coord1[1]]
    node2 = self.graph[coord2[0]][coord2[1]]
    node1.neighbors.append(coord2)
    node2.neighbors.append(coord1)

  def printGraph(self, start, goal):
    for i in range(self.size):
      for j in range(self.size):
        for n in self.graph[i][j].neighbors:
          plt.plot([i, n[0]], [j, n[1]], 'black')
   
    radius = float(self.size)/40
    plt.axis([-1, self.size, -1, self.size])
    begin=plt.Circle(start, radius,color='r')
    end=plt.Circle(goal, radius,color='g')
    plt.gcf().gca().add_artist(begin)
    plt.gcf().gca().add_artist(end)  



g = Graph(22)
start = (random.randint(0, g.size - 1), random.randint(0, g.size - 1))
goal = (random.randint(0, g.size - 1), random.randint(0, g.size - 1))
a = Astar(g.graph, start, goal)
g.printGraph(start, goal)
a.printPath()






                                                      