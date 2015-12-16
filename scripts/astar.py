import matplotlib.pyplot as plt

class Astar():
	def __init__(self, graph, start, goal, viz=False):
		"""	Initialize maze search
				graph: generated from maze.py 
				start: tuple starting coordinate
				goal: tuple goal coordinate
		"""
		self.graph = graph 
		self.start = start 
		self.goal = goal 
		self.viz = viz
		self.frontier = [] #priority queue
		self.came_from = {} #contains previous node
		self.a_star_search()



	def heuristic(self, node):
		""" return calculated manhattan distance from node to goal
				node: node to start from 
		"""
		return abs(node[0]-self.goal[0]) + abs(node[1]-self.goal[1])
	
	def a_star_search(self):
		""" traverse the maze and check nodes by priority until goal is reached
		"""
		#set up starting node
		self.addToQueue(self.start, 0) #starting node with 0 priority
		cost_so_far = {}
		self.came_from[self.start] = None #starting node has no previous node
		cost_so_far[self.start] = 0 #no cost so far
		last = self.start
		while len(self.frontier): #while there are nodes to check
			current = self.frontier.pop()[0] #coordinate with highest priority
			if self.viz:
				self.visualize(current, cost_so_far)
				# plt.plot(current[0], current[1], 'black')
				# plt.gcf().gca().add_artist(checked)


			if current == self.goal: #if we found goal
				print "cost", cost_so_far[current]
				return #done

			x, y = current
			for neighbor in self.graph[x][y].neighbors: #for each neighbor around current node
				newCost = cost_so_far[current] + self.calcWeights(current, neighbor) #calculate new cost

				#if found new node or found a lower cost route to old node
				if neighbor not in cost_so_far or newCost < cost_so_far[neighbor]: 
					cost_so_far[neighbor] = newCost #add/update cost
					priority = -(newCost + self.heuristic(neighbor)) #set priority
					self.came_from[neighbor] = current #add/update previous node
					self.addToQueue(neighbor, priority) #add to Queue

			

	def calcWeights(self, node1, node2):
		"""	return calculated weight moving from node1 to node2
				introduces penalty for turns
				node1: tuple coordinate of first node
				node2: tuple coordinate of second node
		"""
		node3 = self.came_from[node1] #where we came from
		if not node3: #if starting node
			return 1 
		elif node1[0] == node2[0] == node3[0]: #if vertical line
			return 1
		elif node1[1] == node2[1] == node3[1]: #if horizontal line
			return 1 
		else: #turn required
			return 2 #introduce penalty 2 

	def addToQueue(self, node, priority):
		"""	Update Queue with new node and its priority
				output list of nodes sorted by priority
				format [((x, y), z)] where x, y are node coordinates, z is priority
				node: tuple coordinate of node
				priority: integer node priority
		"""
		self.frontier.append((node, priority)) #add to list
		self.frontier.sort(key= lambda prior: prior[1]) #sort by priority

	def visualize(self, current, cost_so_far):
		plt.gca().text(current[0], current[1], str(cost_so_far[current]), fontsize=18)
		last = self.came_from[current]
		if last:
			x1, x2, y1, y2 = (last[0], current[0], last[1], current[1])
			plt.plot([x1, x2], [y1, y2], 'black')
			plt.show(False)
			plt.pause(.01)

#   def visualize(self, current):
#     """ Plot the maze, starting point, ending point, and path using matplotlib
#         Shows a plot
#     """
#     plt.clf()
#     #plot maze
#     for i in range(self.m.size):
#       for j in range(self.m.size):
#         for n in self.m.graph[i][j].neighbors:
#           pd = 0.35
#           #vertical up from (i/n[0],j)
#           if i==n[0] and j < n[1]:
#             rectangle = patches.Rectangle((i-pd, j-pd), 2*pd, 1 + 2*pd, linewidth=0, fc ='w')
#           #vertical up from (i/n[0], n[1])
#           if i==n[0] and j > n[1]:
#             rectangle = patches.Rectangle((i-pd, n[1]-pd), 2*pd, 1 + 2*pd, linewidth=0, fc ='w')
#           #horizontal (i,j/n[1])
#           if j==n[1] and i < n[0]:
#             rectangle = patches.Rectangle((i-pd , j-pd), 1 + 2*pd, 2*pd, linewidth=0, fc ='w')
#           #horizontal (n[0],j/n[1])
#           if j==n[1] and i > n[0]:
#             rectangle = patches.Rectangle((n[0]-pd, j-pd), 1 + 2*pd, 2*pd, linewidth=0, fc ='w')
#           plt.gca().add_patch(rectangle)

#     #plot start and end dots
#     radius = float(self.m.size)/40
#     plt.axis([-1, self.m.size, -1, self.m.size])
#     begin=plt.Circle(self.start, radius, color='r')
#     end=plt.Circle(self.goal, radius,color='g')
#     plt.gcf().gca().add_artist(begin)
#     plt.gcf().gca().add_artist(end)
#     plt.gca().set_axis_bgcolor('black')

#     #plot path
#     for i in range(len(self.path) -1):
#       x1, x2, y1, y2 = (self.path[i][0], self.path[i + 1][0], self.path[i][1], self.path[i + 1][1])
#       plt.plot([x1, x2], [y1, y2], 'red')
      
#     plt.show(False)
#     plt.pause(1)


# if __name__ == "__main__":
# 	size 
# 	start = (0, 0)
#   goal = (random.randint(0, self.m.size - 1), random.randint(0, self.m.size - 1)) #random point in the maze
# 	Astar(start, goal)


