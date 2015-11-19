from Queue import PriorityQueue

class Astar():
	def __init__(self, graph, start, goal):
		self.graph = graph
		self.start = start
		self.goal = goal
	def heuristic(self):
		return abs(start[0]-goal[0]) + abs(start[1]-goal[1])
	def a_star_search(self):
		frontier = PriorityQueue()
		frontier.put(start, 0)
		came_from = {}
		cost_so_far = {}
		came_from[start] = None
		cost_so_far[start] = 0

		while not frontier.empty():
			current = frontier.get()

			if current == goal:
				break

			for next 


 