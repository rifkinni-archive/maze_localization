import numpy as np
import random

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
      self.stack.append(choice)
      self.recursiveBacktracking(choice)
      
    else: #we are blocked in
      if len(self.stack):
        newI, newJ = self.stack.pop()
        Edge(self.graph[newI][newJ], self.graph[i][j])
        self.recursiveBacktracking((newI, newJ))
      else: #we have hit every node in the maze
        print "done"





g = Graph(4)
t = g.test
t.sort()
print 
for i in range(4):
  for j in range(4):
    if g.graph[i][j] == 0:
      print i, j
# print g.graph
# for i in g.graph:
#   for j in g.graph:
#     print g.gr
#     print g.graph[i][j].edges
  # def assignType(self, graph, i, j):
  #   if (i == 0 or i == 6) and (j == 0 or j == 6):
  #     return = Node(i, j, 'edge')    
  #   if (i == 0 or i == 6) and (j == 0 or j == 6):
  #     return = Node(i, j, 'edge')




                                                      