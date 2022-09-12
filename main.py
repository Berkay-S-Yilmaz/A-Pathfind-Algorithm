#Module Imports
import pygame, math
from queue import PriorityQueue

#Creatin Main Window. The size, shape(square), and Title
WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")

#Color Codes Variables that'll be used in the program
#All caps naming convention to indicate constant
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREY = (128, 128, 128)
RED = (255, 0, 0)
ORANGE = (255, 165 ,0)
YELLOW = (255, 255, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
PURPLE = (128, 0, 128)
TURQUOISE = (64, 224, 208)

#Building Visualisation class.
class Node:
	#initialising attributes
	def __init__(self, row, col, width, all_rows):
		self.row = row
		self.col = col
		self.width = width
		self.all_rows = all_rows
		self.x = row * width
		self.y = col * width
		#setting base color to white/blank
		self.color = WHITE
		self.neighbors = []
		
	#Method to gets the position e.g row 10, col 5
	def get_pos(self):
		#Indexing using row columns e.g y:6, x:11
		return self.row, self.col

#--------Color Coding to display information--------

#Resetting map color back to blank
	def reset(self):
		self.color = WHITE

#Methods for start and finish points
	def is_start(self):
		return self.color == ORANGE
	def is_end(self):
		return self.color == TURQUOISE

#Method that turn red when a node is passed/closed and no longer a path option
	def is_closed(self):
		return self.color == RED
#Method to display open points/nodes that haven't been used
	def is_open(self):
		return self.color == GREEN

#Method for coloring of obstacles algorithm must avoid
	def is_barrier(self):
		return self.color == BLACK


#--------Making the colors change--------
	def make_is_start(self):
		self.color = ORANGE
	def make_is_closed(self):
		self.color = RED
	def make_is_open(self):
		self.color = GREEN
	def make_is_barrier(self):
		self.color = BLACK
	def make_is_end(self):
		self.color = TURQUOISE
	def make_is_path(self):
		self.color = PURPLE

    #Method to draw everything in Window
	def draw(self, win):
		pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

	#Preventing barriers from blocking in, the pathfinder
	def update_neighbors(self, grid):
		self.neighbors = []
		#checking if there is space beneath/down, then add it as a neighboring grid
		if self.row < self.all_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): 
			self.neighbors.append(grid[self.row + 1][self.col])
		#Above/Up
		if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():
			self.neighbors.append(grid[self.row - 1][self.col])
		#Left-side
		if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): 
			self.neighbors.append(grid[self.row][self.col - 1])
		# Right-side
		if self.col < self.all_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): 
			self.neighbors.append(grid[self.row][self.col + 1])

	
	#Less than, two spot objects to determine which one is less than
	def __lt__(self, other):
		return False

#Heuristic function to discover Manhattan Distance between start & end nodes
def heuristic(p1, p2):
	x1, y1 = p1
	x2, y2 = p2
	return abs(x1 - x2) + abs(y1 - y2)
	#E.g start == 1,1 & End == 9,9. Distance == 8,8

def reconstruct_path(came_from, current, draw):
	while current in came_from:
		current = came_from[current]
		current.make_is_path()
		draw()

def algorithm(draw, grid, start, end):
	count = 0
	open_set = PriorityQueue()
	#add start node with fscore set to 0
	#count keeps track of when items were inserted into queue. in the event of tied fscores, select the first option
	open_set.put((0, count, start))
	#tracks the path e.g node b came from node a
	came_from = {}
	#Dictionary comprehensions for g and f scores
	g_score = {node: float("inf") for row in grid for node in row}
	g_score[start] = 0
	f_score = {node: float("inf") for row in grid for node in row}
	#fscore is heuristic to estimate the distance between start and end node
	f_score[start] = heuristic(start.get_pos(), end.get_pos())
	
	open_set_hash = {start}

	#algorithm runs until open set is empty
	while not open_set.empty():
		for event in pygame.event.get():
			#exit loop if application is quit
			if event.type == pygame.QUIT:
				pygame.quit()

		#indexing open_set start
		current = open_set.get()[2]
		open_set_hash.remove(current)

		#Condition to draw path once the end is reached
		if current == end:
			reconstruct_path(came_from, end, draw)
			end.make_is_end()
			return True

		#Neighbors of the current node by adding +1 cube/block
		for neighbor in current.neighbors:
			temp_g_score = g_score[current] + 1

			#Determining optimal path by comparing gscore between neighboring blocks
			if temp_g_score < g_score[neighbor]:
				#updating where the node came from
				came_from[neighbor] = current
				#updating current scores to determine current best path 
				g_score[neighbor] = temp_g_score
				f_score[neighbor] = temp_g_score + heuristic(neighbor.get_pos(), end.get_pos())
				#if the neighbor isn't in the open set incrament it in
				if neighbor not in open_set_hash:
					count += 1
					#put in new neighbor and consider its path
					open_set.put((f_score[neighbor], count, neighbor))
					open_set_hash.add(neighbor)
					#make the current neighbor open as it was put into the openset (openset are nodes that were viable options)
					neighbor.make_is_open()
	
		draw()
		#if the current node, isn't the start node, close it off as it isn't in open set
		if current != start:
			current.make_is_closed()
	return False

#Function to make the grid 
def make_grid(rows, width):
	grid = []
	gap = width // rows
	for _ in range(rows):
		grid.append([])
		for __ in range(rows):
			node = Node(_, __, gap, rows)
			grid[_].append(node)
	return grid

#Drawing the gird lines
def draw_grid(win, rows, width):
	gap = width // rows
	for _ in range(rows):
		#for everyline draw horizontal row
		pygame.draw.line(win, GREY, (0, _ * gap), (width, _ * gap))
		for __ in range(rows):
			#for everyline draw vertical line
			pygame.draw.line(win, GREY, (__ * gap, 0), (__ * gap, width))

#The main draw function that draws everything
def draw(win, grid, rows, width):
	#fills everything white
	win.fill(WHITE)

	for row in grid:
		for node in row:
			node.draw(win)

	#Update the drawings on display
	draw_grid(win, rows, width)
	pygame.display.update()

#find the position of the spot being clicked
def get_clicked_pos(pos, rows, width):
	gap = width // rows
	#Getting y & x position 
	y, x = pos
	#dividng by the width of each cube to get the exact location
	row = y // gap
	col = x // gap
	return row, col

#Main loop to run application
def main(win, width):
	#Row object to set the amount of cubes in grid
	ROWS = 50
	grid = make_grid(ROWS, width)

	#Start and end nodes set to none initially as the user decides position
	start = None
	end = None

	run = True
	while run is True:
		draw(win, grid, ROWS, width)
		#Looping through all events and checking them
		for event in pygame.event.get():
			#condition to break the loop (run no longer meets condition)
			if event.type == pygame.QUIT:
				run = False
			
			#Condition for Left Mouse Click (the drawing button)
			if pygame.mouse.get_pressed()[0]: 
				#getting position of the cube/s clicked by left mouse
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, width)
				node = grid[row][col]
				#condition to make the first two clicked cubes, the start and end nodes
				if not start and node != end:
					start = node
					#making start node
					start.make_is_start()
				#making end node
				elif not end and node != start:
					end = node
					end.make_is_end()
				elif node != end and node != start:
					#once start & end nodes are made, the other clicked nodes are barriers
					node.make_is_barrier()

			#Conditionn for Right Mouse Click (Erasing drawn objects)
			elif pygame.mouse.get_pressed()[2]: 
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, width)
				node = grid[row][col]
				node.reset()
				if node == start:
					#Erasing by changing start to None, making it WHITE
					start = None
				elif node == end:
					end = None

			#If spacekey is pressed, update all the rows in the gird, update the neighboring blocks
			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_SPACE and start and end:
					for row in grid:
						for node in row:
							node.update_neighbors(grid)
					#annonymous function, draw function passed as argument, then grid and start and end points
					algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)

				if event.key == pygame.K_c:
					start = None
					end = None
					grid = make_grid(ROWS, width)
	#when run is no longer true, then quit the application
	pygame.quit()
main(WIN, WIDTH)