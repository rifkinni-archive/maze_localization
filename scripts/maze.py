import numpy as np
import random
from matplotlib import patches, pyplot as plt
import time


class Node(object):
  """ A node in the graph
      contains:
        position on graph x and y
        reference to other connected nodes "neighbors"
  """
  def __init__(self, x, y):
    self.coordX = x #position on a grid
    self.coordY = y 
    self.neighbors = []

class Maze(object):
  """ builds a maze
  """
  def __init__(self, size, viz=False):
    """ size: square length of the maze
        viz: optional parameter to visualize the maze as it is generated
    """
    self.viz = viz
    self.size = size
    self.show = True
    self.graph = [[0 for x in range(self.size)] for x in range(self.size)]
    self.stack = [(0,0)]
    self.recursiveBacktracking((0, 0))
    self.addEdges()
    

  def getUnvisitedNodes(self, i, j):
    """ returns choices of neighboring nodes 
        that have not been visited by recursiveBacktracking 
        in a random order
    """
    adjacentInd = [(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)] #adjacent node indices
    choices = []
    for x, y in adjacentInd:
      if (x >= 0) and (y >=0):
        if (x < self.size) and (y < self.size): #if within our graph
          if self.graph[x][y] == 0: #not already visited
            choices.append((x, y))
    random.shuffle(choices)
    return choices

  def getUnconnectedNeighbors(self, i, j):
    """ returns choices of neighboring nodes
        that are not connected to node i, j for addEdges
        in a random order

      things to test:
        returns array of length between 2 and 4
        if length 2 it i, j is a corner (i or j has only 0 or self.size - 1)
        if length 3, i, j is an edge (i or j has a 0 or a self.size -1)
    """
    adjacentInd = [(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)] #adjacent node indices
    choices = []
    for x, y in adjacentInd:
      if (x >= 0) and (y >=0):
        if (x < self.size) and (y < self.size): #if within our graph
          choices.append((x, y))
    
    choices = [a for a in choices if a not in self.graph[i][j].neighbors] #not already a connection
    random.shuffle(choices)
    return choices

  def recursiveBacktracking(self, indices):
    """ Recursively connects nodes in the maze
        indices: tuple of node indices (tracks current position)
        end product is a maze with exactly one path from a to b

      things to test:
        every node has at least one neighbor
        every node is a node object (not 0)
    """
    i, j = indices
    choices = self.getUnvisitedNodes(i, j) 

    if self.graph[i][j] == 0:
      self.graph[i][j] = Node(i, j) #create node once we've visited
      
    if choices: #we are not blocked in
      choice = choices[0] #first of a random list
      self.stack.append((i, j))

      self.recursiveBacktracking(choice)
      
    else: #we are blocked in
      if len(self.stack):
        newI, newJ = self.stack.pop() #highest priority in the stack
        self.updateNeighbors((newI, newJ), (i, j))
        if self.viz:
          self.visualize()
        self.recursiveBacktracking((newI, newJ))
      #else we have hit every node in the maze

  def updateNeighbors(self, coord1, coord2):
    """ coord1: first neighbor's coordinates
        coord2: second neighbor's coordinates
        Update neighbors list for nodes when edge is added

      things to test:
        node1 is node2's neighbor
        node2 is node1's neighbor
    """

    node1 = self.graph[coord1[0]][coord1[1]]
    node2 = self.graph[coord2[0]][coord2[1]]
    node1.neighbors.append(coord2)
    node2.neighbors.append(coord1)

  def addEdges(self):
    """ Adds maze size - 2 additional connections 
        so that there is more than one path to the end

    """
    counter = 0
    while True:
      x, y = [random.randint(0, self.size - 1), random.randint(0, self.size - 1)] #random coordinates
      available = self.getUnconnectedNeighbors(x, y) #if it has available neighbors
      if available:
        self.updateNeighbors((x, y), available[0])
        counter +=1
        if self.viz:
          self.visualize()
      if counter >= self.size - 2: #add size - 2 edges
        break

  def visualize(self):
    """ Plot the maze using matplotlib as it is generated
        Shows a plot
    """
    plt.clf()
    #plot maze
    for i in range(self.size):
      for j in range(self.size):
        if self.graph[i][j]:
          for n in self.graph[i][j].neighbors:
            pd = 0.35
            #vertical up from (i/n[0],j)
            if i==n[0] and j < n[1]:
              rectangle = patches.Rectangle((i-pd, j-pd), 2*pd, 1 + 2*pd, linewidth=0, fc ='w')
            #vertical up from (i/n[0], n[1])
            if i==n[0] and j > n[1]:
              rectangle = patches.Rectangle((i-pd, n[1]-pd), 2*pd, 1 + 2*pd, linewidth=0, fc ='w')
            #horizontal (i,j/n[1])
            if j==n[1] and i < n[0]:
              rectangle = patches.Rectangle((i-pd , j-pd), 1 + 2*pd, 2*pd, linewidth=0, fc ='w')
            #horizontal (n[0],j/n[1])
            if j==n[1] and i > n[0]:
              rectangle = patches.Rectangle((n[0]-pd, j-pd), 1 + 2*pd, 2*pd, linewidth=0, fc ='w')
            plt.gca().add_patch(rectangle)


    plt.axis([-1, self.size, -1, self.size])
    plt.gca().set_axis_bgcolor('black')
    plt.show(False)
    if self.show:
      time.sleep(3)
      self.show = False
    plt.pause(0.01)

if __name__ == "__main__":
  m = Maze(22, viz=True)

                                       

