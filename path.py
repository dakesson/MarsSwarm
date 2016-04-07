#!/usr/bin/python
from __future__ import print_function
import collections
import heapq
from field import * 

class Queue:
    def __init__(self):
        self.elements = collections.deque()
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, x):
        self.elements.append(x)
    
    def get(self):
        return self.elements.popleft()

#class SimpleGraph:
 #   def __init__(self):
  #      self.edges = {}
    
    #def neighbors(self, id):
     #   return self.edges[id]

def draw_tile(graph, id, style, width):
    r = "."
    if 'number' in style and id in style['number']: r = "%d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = "\u2192"
        if x2 == x1 - 1: r = "\u2190"
        if y2 == y1 + 1: r = "\u2193"
        if y2 == y1 - 1: r = "\u2191"
    if 'start' in style and id == style['start']: r = "A"
    if 'goal' in style and id == style['goal']: r = "Z"
    if 'path' in style and id in style['path']: r = "@"
    if id in graph.walls: r = "#" * width
    return r

def draw_grid(graph, width=2, **style):
    for y in range(graph.height):
        for x in range(graph.width):
            print("%%-%ds" % width % draw_tile(graph, (x, y), style, width),end="")
        print()
        

        
def breadth_first_search(graph, start, goal):
    frontier = Queue()
    frontier.put(start)
    came_from = {}
    came_from[start] = None
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current,1):
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current
    
    return came_from
def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
    def smallest(self):
        return 

def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    #print("start is "+str(start))
    #print("goal is "+str(goal))
    #print("came_from is "+str(came_from))
    try:#try if path can be found
        while current != start:
            #print("current is "+str(current))
            current = came_from[current]
            path.append(current)
            #print("path is "+str(path))
        path.reverse()
        return path
    except:#path can not be found, possibly blocked by another robot
        return None
    

        
def a_star_search(graph, start, goal,robot):
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
        
        for next in graph.neighbors(current,1):
            new_cost = cost_so_far[current] + graph.cost(current, next, robot)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

def from_id_width(id, width):
    return (id % width, id // width)
    
#DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434]]

#g = SquareGrid(30, 15)
#g.walls = DIAGRAM1_WALLS

#parents = breadth_first_search(g, (8, 7),(17,2))
#print(parents)
#draw_grid(g, width=2, point_to=parents, start=(8, 7), goal=(17, 2))
#print()


#print("A*")

        
#diagram4 = GridWithWeights(40, 40)
#diagram4.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
#diagram4.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
 #                                      (4, 3), (4, 4), (4, 5), (4, 6), 
  #                                     (4, 7), (4, 8), (5, 1), (5, 2),
   ##                                    (5, 3), (5, 4), (5, 5), (5, 6), 
     #                                  (5, 7), (5, 8), (6, 2), (6, 3), 
      #                                 (6, 4), (6, 5), (6, 6), (6, 7), 
       #                                (7, 3), (7, 4), (7, 5)]}
#diagram4.elevation = {(0,0):0,(0,1):0,(0,2):0,(0,3):0,(0,4):0,(0,5):0,(0,6):0,(0,7):0,(0,8):0,(0,9):0,(1,0):0,(1,1):0,(1,2):0,(1,3):0,(1,4):0,(1,5):0,(1,6):0,(1,7):20,(1,8):20,(1,9):0,(2,0):0,(2,1):0,(2,2):0,(2,3):0,(2,4):0,(2,5):0,(2,6):0,(2,7):20,(2,8):20,(2,9):0,(3,0):0,(3,1):0,(3,2):0,(3,3):0,(3,4):0,(3,5):0,(3,6):0,(3,7):20,(3,8):20,(3,9):0,(4,0):0,(4,1):0,(4,2):0,(4,3):0,(4,4):0,(4,5):0,(4,6):0,(4,7):0,(4,8):0,(4,9):0,(5,0):0,(5,1):0,(5,2):0,(5,3):0,(5,4):0,(5,5):0,(5,6):0,(5,7):0,(5,8):0,(5,9):0,(6,0):0,(6,1):0,(6,2):0,(6,3):0,(6,4):0,(6,5):0,(6,6):0,(6,7):0,(6,8):0,(6,9):0,(7,0):0,(7,1):0,(7,2):0,(7,3):0,(7,4):0,(7,5):0,(7,6):0,(7,7):0,(7,8):0,(7,9):0,(8,0):0,(8,1):0,(8,2):0,(8,3):0,(8,4):0,(8,5):0,(8,6):0,(8,7):0,(8,8):0,(8,9):0,(9,0):0,(9,1):0,(9,2):0,(9,3):0,(9,4):0,(9,5):0,(9,6):0,(9,7):0,(9,8):0,(9,9):0}
#diagram4.getElevation("elevation.png")                                      
                                       
#came_from, cost_so_far = a_star_search(diagram4, (18, 36), (9, 4))
#draw_grid(diagram4, width=3, point_to=came_from, start=(18, 36), goal=(9, 4))
#print()
#draw_grid(diagram4, width=3, number=cost_so_far, start=(18, 36), goal=(9, 4))
#print()
#draw_grid(diagram4, width=3, path=reconstruct_path(came_from, start=(18, 36), goal=(9, 4)))