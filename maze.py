import numpy as np
import random

class Node(object):
  def __init__(self, x, y):
    # self.nodeType = nodeType
    self.priority = 0
    self.coordX = x #position on a grid
    self.coordY = y 
    self.edges = None



class Edge(object):
  def __init__(self, node1, node2):
    self.weight = 0
    self.node1 = node1
    self.node2 = node2
    encodedInstructions = None
  def encodeInstructionsByOrientation(self, orientation):
    pass



class Graph(object):
  def __init__(self, size):
    self.size = size
    self.graph = [[0 for x in range(self.size)] for x in range(self.size)]
    self.stack = []
    self.recursiveBacktracking((0, 0))
    # for i in range(size):
    #   for j in range(size):
    #     graph[i][j] = self.assignType(self)

  def getNodeChoices(self, i, j):
    
    directions = [(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)]
    choices = []
    for x, y in directions:
      if (x >= 0) and (y >=0):
        if (x < self.size) and (y < self.size):
          if self.graph[x][y] == 0:
            choices.append((x, y))
    random.shuffle(choices)
    # if len(choices):
    #   choice = random.choice(choices)
    return choices

  def recursiveBacktracking(self, indices):
    i, j = indices
    choice = self.getNodeChoices(i, j)[0]
    if choice:
      if self.graph[i][j] == 0:
        self.stack.append((i, j))
        self.graph[i][j] = Node(i, j)
        self.recursiveBacktracking(choice)
      else:
        pass
      
    else:
      self.recursiveBacktracking(self.stack.pop)




g = Graph(6)
print g.graph
  # def assignType(self, graph, i, j):
  #   if (i == 0 or i == 6) and (j == 0 or j == 6):
  #     return = Node(i, j, 'edge')    
  #   if (i == 0 or i == 6) and (j == 0 or j == 6):
  #     return = Node(i, j, 'edge')





c v                                                       