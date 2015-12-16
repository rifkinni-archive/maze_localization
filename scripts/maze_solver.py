from matplotlib import patches, pyplot as plt
import math
import random
from maze import Maze
from astar import Astar
import random

class MazeSolver():
  """ A class that provides: 
        a path through a maze
        robot instructions to navigate it
        a visualization of the solved maze
  """
  def __init__(self):
    self.m = Maze(10)

    self.start = (0, 0)
    self.goal = (random.randint(0, self.m.size - 1), random.randint(0, self.m.size - 1)) #random point in the maze
    self.visualizeAstar()
    self.a = Astar(self.m.graph,self.start, self.goal, viz=True) #solve maze using astar
    self.path = self.getPath()
    # self.visualizeAstarPath()
    self.instructions = self.getInstructions()


  def getInstructions(self):
    """ get turn instructions for the robot to execute 
        note: the robot will always move forward one unit, so no custom instructions are needed
        returns a list of tuples of form: 
          [(turn in radians, 
          orientation of the robot after the turn, 
          human readable instruction e.g. "right"), ...]
    """
    instructions = []
    for i in range(1, len(self.path)):

      if i == 1: #first instruction
        orientation = 0 #assume robot facing 0 degrees
      else:
        orientation = instructions[i-2][1] #last robot orientation

      nextNode = self.path[i] 
      currentNode = self.path[i-1]

      nextOrient = self.getNextOrientation(currentNode, nextNode) 
      turn = self.getTurn(orientation, nextOrient)
      instructions.append((turn[0], nextOrient, turn[1]))

    return instructions

  def getNextOrientation(self, currentNode, nextNode):
    """ get orientation of robot after the turn
        currentNode: coordinates of current node
        nextNode: coordinates of next node
        returns one of [0, 1, 2, 3]
    """
    if nextNode[0] == currentNode[0]:  # x coordinates are equal
        nextOrient = 0 if nextNode[1] > currentNode[1] else 2 #forward or backwards

    else: #y coordinates are equal
      nextOrient = 1 if nextNode[0] > currentNode[0] else 3 #left or right

    return nextOrient

  def getTurn(self, currentOrient, nextOrient):
    """ get the turn angle depending on the change in orientation
        current: current orientation, one of [0, 1, 2, 3]
        _next: nex orientation, one of [0, 1, 2, 3]
        return tuple of form:
          (turn in radians, 
          human readable instruction e.g. "right")
    """
    case = (nextOrient - currentOrient)%4 #difference in orientations

    if case == 0: #same orientation as before
      return 0, "no turn"

    elif case == 1: #right turn
      return -math.pi/2, "right"

    elif case == 2: #180 turn
      return math.pi, "full"
    
    elif case == 3: #left turn
      return math.pi/2, "left"    


  def getNeighbors(self, coord):
    """ gets node neighbors (nodes we are connected to by graph edges)
        coord: coordinates of node 
        returns list of neighbors
    """
    x = coord[0]
    y = coord[1]
    return self.m.graph[x][y].neighbors

  def getPath(self):
    """ get nodes in order of traversal
        goal: end coordinate of the maze
        returns list of node coordinates
    """
    path = []

    node = self.goal #work backwards
    last = self.a.came_from[node] #node we came from
    
    while last: #not at starting node
      path.append(node)
      node = last #update current node
      last = self.a.came_from[last]
    path.append(node) #add the last node
    return list(reversed(path)) #reverse since we started from goal

  def visualizeAstar(self):
    plt.clf()
    #plot maze
    for i in range(self.m.size):
      for j in range(self.m.size):
        for n in self.m.graph[i][j].neighbors:
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
    radius = float(self.m.size)/40
    plt.axis([-1, self.m.size, -1, self.m.size])
    begin=plt.Circle(self.start, radius, color='r')
    end=plt.Circle(self.goal, radius,color='g')
    plt.gcf().gca().add_artist(begin)
    plt.gcf().gca().add_artist(end)
    plt.gca().set_axis_bgcolor('black')
    plt.show(False)
    plt.pause(0.01)

  # def visualizeAstarPath(self):
  #   self.visualizeAstar()
  #   for i in range(len(self.path) -1):
  #     x1, x2, y1, y2 = (self.path[i][0], self.path[i + 1][0], self.path[i][1], self.path[i + 1][1])
  #     plt.plot([x1, x2], [y1, y2], 'red')
  #     plt.show(False)
  #     plt.pause(0.01)

  def visualize(self, current):
    """ Plot the maze, starting point, ending point, and path using matplotlib
        Shows a plot
    """
    plt.clf()
    #plot maze
    for i in range(self.m.size):
      for j in range(self.m.size):
        for n in self.m.graph[i][j].neighbors:
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
    radius = float(self.m.size)/40
    plt.axis([-1, self.m.size, -1, self.m.size])
    begin=plt.Circle(self.start, radius, color='r')
    end=plt.Circle(self.goal, radius,color='g')
    location=plt.Circle(current, radius, color='k')
    plt.gcf().gca().add_artist(begin)
    plt.gcf().gca().add_artist(end)
    plt.gcf().gca().add_artist(location)
    plt.gca().set_axis_bgcolor('black')

    #plot path
    for i in range(len(self.path) -1):
      x1, x2, y1, y2 = (self.path[i][0], self.path[i + 1][0], self.path[i][1], self.path[i + 1][1])
      plt.plot([x1, x2], [y1, y2], 'red')

    plt.show(False)
    plt.pause(0.01)



if __name__ == '__main__':

  solver = MazeSolver()
  for node in solver.path:
    solver.visualize(node)


  def tester(current, _next):
    """ This was an original brute force function for determining orientation
        It is used to compare results and test with a much cleaner and more compact function
    """
    if current == _next:
      return 0, "no turn"

    elif abs(current - _next) == 2:
      return math.pi, "full"

    elif current == 1  or current == 2:
      if _next - current > 0:
        return -math.pi/2, "right"
      else:
        return math.pi/2, "left"
    elif current == 0:
      if _next == 1:
        return -math.pi/2, "right"
      else:
        return math.pi/2, "left"
    elif current == 3:
      if _next == 0:
        return -math.pi/2, "right"
      else:
        return math.pi/2, "left"

  #unit testing
  for i in [0, 1, 2, 3]:
    for j in [0, 1, 2, 3]:
      _, a = solver.getTurn(i, j)
      _, b = tester(i, j)
      assert a == b


