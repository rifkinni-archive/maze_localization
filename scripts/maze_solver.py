import matplotlib.pyplot as plt
from matplotlib import patches
import numpy as np
import random
from maze import Graph
from astar import Astar

class MazeSolver():
  def __init__(self):
    self.g = Graph(10)

    start = (0, 0)
    goal = (random.randint(0, self.g.size - 1), random.randint(0, self.g.size - 1))

    self.a = Astar(self.g.graph,start, goal)
    self.path = self.getPath(goal)
    self.visualize(start, goal)


  def getInstructions(self):
    inst = []
    orientation = 0
    tup = [0, 0]
    for i in range(1, len(self.path)):
      tup = [0, 0]

      if i == 1:
        orientation = 0
      else:
        orientation = inst[i-2][1]

      _next = self.path[i]
      current = self.path[i-1]
      # x is equal
      if _next[0] == current[0]:
        orient = 0 if _next[1] > current[1] else 2
        tup[0] = self.getTurn(orientation, orient)
        tup[1] = orient

      else:
        orient = 1 if _next[0] > current[0] else 3
        tup[0] = self.getTurn(orientation, orient)
        tup[1] = orient
        
      inst.append(tup)

    return inst

  def getTurn(self, first, second):
    """ get the instruction depending on the orientation """
    if abs(second - first) == 2:
      return "full"
    if second == first:
      return "no turn"
    elif second - first > 0:
      return "right"
    elif first - second > 0:
      return "left"

  def getNeighbors(self, coord):
    x = coord[0]
    y = coord[1]
    return self.g.graph[x][y].neighbors

  def getPath(self, goal):
    path = []

    node = goal #work backwards
    last = self.a.came_from[node] #node we came from
    
    while last: #not at starting node
      path.append(node)
      node = last #update current node
      last = self.a.came_from[last]
    path.append(node) #add the last node
    return list(reversed(path))


  def visualize(self, start, goal):
    """Plot the maze, start, end, and path using matplotlib"""
    #plot maze
    for i in range(self.g.size):
      for j in range(self.g.size):
        for n in self.g.graph[i][j].neighbors:
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

    #plot start and end dots
    radius = float(self.g.size)/40
    plt.axis([-1, self.g.size, -1, self.g.size])
    begin=plt.Circle(start, radius, color='r')
    end=plt.Circle(goal, radius,color='g')
    plt.gcf().gca().add_artist(begin)
    plt.gcf().gca().add_artist(end)
    plt.gca().set_axis_bgcolor('black')

    #plot path
    for i in range(len(self.path) -1):
      x1, x2, y1, y2 = (self.path[i][0], self.path[i + 1][0], self.path[i][1], self.path[i + 1][1])
      plt.plot([x1, x2], [y1, y2], 'red')

    plt.show()

if __name__ == '__main__':
  x = MazeSolver()