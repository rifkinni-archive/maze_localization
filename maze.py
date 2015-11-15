import numpy as np
import random
import matplotlib.pyplot as plt


class Node(object):
  def __init__(self, x, y):
    # self.nodeType = nodeType
    self.priority = 0
    self.coordX = x #position on a grid
    self.coordY = y 
    self.edges = []



class Edge(object):
  def __init__(self, node1, node2):
    self.weight = 0
    self.node1 = node1
    self.node2 = node2
    self.nodeReference()
    encodedInstructions = None
  def nodeReference(self):
    self.node1.edges.append(self)
    self.node2.edges.append(self)
  def encodeInstructionsByOrientation(self, orientation):
    pass



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
        Edge(self.graph[newI][newJ], self.graph[i][j])
        self.recursiveBacktracking((newI, newJ))
      else: #we have hit every node in the maze
        print "done"

  def printGraph(self):
    for i in range(self.size):
      for j in range(self.size):
        print i,j
        for n in self.graph[i][j].edges:
          plt.plot([n.node1.coordX, n.node2.coordX], [n.node1.coordY, n.node2.coordY])

    plt.axis([-1, self.size, -1, self.size])
    plt.show()    


g = Graph(22)
g.printGraph()




                                                      