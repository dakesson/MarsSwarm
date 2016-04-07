#!/usr/bin/python
import math,pygame
import collections
from path import *
import numpy as np
import numpy.linalg as la
from PIL import Image
import re

class Vector:
    def __init__(self, a, b):
        self.X = a
        self.Y = b
   
    def unitize(self):
        if(math.pow(self.X,2)+math.pow(self.Y,2)!=0):
            self.X = self.X/math.sqrt(math.pow(self.X,2)+math.pow(self.Y,2))
            self.Y = self.Y/math.sqrt(math.pow(self.X,2)+math.pow(self.Y,2))
        else:
            self.X = 0
            self.Y = 0
        return self
    ##rotate vector counter clockwise
    def rotate(self,r):
        self.X = self.X*math.cos(r)-self.Y*math.sin(r)        
        self.Y = self.X*math.sin(r)+self.Y*math.cos(r)        
        return self

    def angle(self,v1):
        cosang = np.dot([self.X,self.Y], [v1.X,v1.Y])
        sinang = la.norm(np.cross([self.X,self.Y], [v1.X,v1.Y]))
        return np.arctan2(sinang, cosang)
    
    def __add__(self,other):
        return Vector(self.X + other.X, self.Y + other.Y)
    
    def __sub__(self,other):
        return Vector(self.X - other.X, self.Y - other.Y)
    
    def __mul__(self, scalar):
        return Point(self.X*scalar, self.Y*scalar)
    def __div__(self, scalar):
        return Point(self.X/scalar, self.Y/scalar)
    def __len__(self):
        return round(math.sqrt(self.X**2 + self.Y**2))
    def get(self):
        return (self.X, self.Y)
        
class Point:
    def __init__(self, a, b):
        self.X = a
        self.Y = b
    def distance(self,b):
        return math.sqrt(math.pow(self.X-b.X,2)+math.pow(self.Y-b.Y,2))
    def get(self):
        return (self.X, self.Y)
   
class SquareGrid:#walls are not used. Simply use elevation to generate cost for moving
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
    
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id):
        return id not in self.walls
    
    def neighbors(self, id, radius):
        (x, y) = id
        results = []
        for h in range(-1*radius+x,radius+x+1):
            for v in range(-1*radius+y,radius+y+1):
                if((h,v)is not (x,y)):
                    results.append((h,v))
        #results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1),(x+1,y+1),(x+1,y-1),(x-1,y+1),(x-1,y-1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

class GridWithWeights(SquareGrid):#use elevation to generate cost for moving
    def __init__(self, width, height):
        #super().__init__(width, height)#python3
        SquareGrid.__init__(self,width, height)
        self.weights = {}
        self.elevation = {}
        self.scent = {}
        for h in range(width):
            for v in range(height):
                self.scent[(h,v)]=0
        #print(self.scent)
    
    def evaporateScent(self):
        for x in range(self.width):
            for y in range(self.height):
                self.scent[(x,y)]= max(self.scent.get((x,y),0)-0.05,0)


        
    def getElevationFromImage(self,elevationMap,field):#get elevation from image
        im = Image.open(elevationMap)
        im = im.convert('RGB')
        
        imageWith,imageHeight = im.size
        for x in range(self.width):
            for y in range(self.height):
                #R,G,B = pix[x, y]
                R,G,B = im.getpixel((x,y))
                self.elevation[(x,y)]= ((field.distanceMax-field.distanceMin)/field.distanceStep)*(sum([R,G,B])/3)/255
    
    def getElevationJson(self,field,robots,elevationJson):#get elevation from Json file, distance from camera used in json, need to translate to elevation from ground
        #remove elevation from robot location
        robotGrid = []
        for rob in robots:
            in_first = set(robotGrid)
            in_second = set(list(field.weightedGrid.neighbors(rob.positionInGrid,3)))

            in_second_but_not_in_first = in_second - in_first
            robotGrid = robotGrid + in_second_but_not_in_first
        for key in elevationJson.keys():
            #xy = re.split("(|,|)",key)
            #key = (xy[0],xy[1])
            xy = key[1:-1].split(",")
            keyInGrid = (int(xy[0]),int(xy[1]))

            if(elevationJson[key]["e"]>field.distanceMax):#Minimum elevation is 0, the maximum is (field.distanceMax - field.distanceMin)/field.distanceStep
                self.elevation[keyInGrid] = 0
            elif(elevationJson[key]["e"]<field.distanceMin):
                self.elevation[keyInGrid] = (field.distanceMax - field.distanceMin)/field.distanceStep
            else:
                self.elevation[keyInGrid] = (field.distanceMax -elevationJson[key]["e"])/field.distanceStep


    def cost(self,from_node, to_node,robot):#cost moving on flat surface is 1 per grid. Cost on tilted surface is the difference between elevations
        loaded = robot.isSandloaded
        field = robot.field
        
        if(field.weightedGrid.scent.get(to_node,1)>0 and loaded==False): # add a cost for moving on pheromone trails
            self.scentfactor=10
        else:
            self.scentfactor=1
            
        if(self.elevation.get(to_node, 1)==self.elevation.get(from_node, 1)):
            return 1*self.scentfactor
        else:
            return 4*abs(self.elevation.get(to_node, 1)-self.elevation.get(from_node, 1))*self.scentfactor

    
    def neighborElevation(self, id):#find elevation near current position in a 5*5 grid
        (x, y) = id
        radius = 3
        results = self.neighbors(id,radius)
        # #for h in range(-1*radius+x,radius+x):
            # for v in range(-1*radius+y,radius+y):
                # results.append((h,v))
        # if (x + y) % 2 == 0: results.reverse() # aesthetics
        # results = filter(self.in_bounds, results)
        # results = filter(self.passable, results)
        foundNeighborElevation = {}
        foundNeighbor = {}
        for count in results:
            foundNeighborElevation[count]=self.elevation[count]
            foundNeighbor[count] = 1
        return foundNeighbor,foundNeighborElevation
            
class Field:
    def __init__(self, x, y, g):
        self.width = x
        self.height = y
        self.gridSize = g
        self.gridWidth = int(x/g)
        self.gridHeight = int(y/g)
        self.distanceMin = 1000#min depth for kinect
        self.distanceMax = 1800#max depth for kinect
        self.distanceStep = 20#The absolute distance difference is 200mm, with step 20mm, this means climing up 20mm has the same cost as travel one grid size(for a 3000*2400 field and 50*40 grid, grid size is 60mm) 
        self.maxElevation = (self.distanceMax-self.distanceMin)/self.distanceStep
        self.weightedGrid = GridWithWeights(self.gridWidth,self.gridHeight)
        self.targetGridElevation = GridWithWeights(self.gridWidth,self.gridHeight)
        self.targetQueue = {}
        self.actualFieldSize2DisplayField = 6#translate from screen resolution to actual resolution
        self.simulation = False
        self.busyTargets = []
        
    def setTarget(self,t):
        self.target = t
        self.targetInGrid = (math.floor(t.c.X/self.gridSize),math.floor(t.c.Y/self.gridSize))
    def setLoadingArea(self,l):
        self.loadingArea = l
        self.loadingAreaInGrid = (math.floor(l.c.X/self.gridSize),math.floor(l.c.Y/self.gridSize))
    def grid2Display(self,t):
        (x,y) = t
        return (x*self.gridSize,y*self.gridSize)
    def gridDistance2ScreenDistance(self,grid1,grid2):
        (grid1X,grid1Y) = grid1
        (grid2X,grid2Y) = grid2
        return Point(grid1X,grid1Y).distance(Point(grid2X,grid2Y))
    def calculateTargetQueue(self):
        print("Field calculate")
        self.targetQueue = {}
#        MINIMUM_TARGET_LOADING_DISTANCE = 40
        for key in self.weightedGrid.elevation.keys():
            self.targetQueue[key]=self.weightedGrid.elevation[key]-self.targetGridElevation.elevation[key]
#        self.orderedTargetQueue = collections.OrderedDict(sorted(self.targetQueue.items(), key=lambda t: t[1]))
        #print("Ordered queue"+str(orderedTargetQueue))
        #print("Target changed"+str(orderedTargetQueue.popitem(last=False)[0]))
        #print("Target changed"+str(orderedTargetQueue.popitem(last=True)[0]))
#        (targetInGridX,targetInGridY) = orderedTargetQueue.popitem(last=False)[0]
#        (targetInScreenX,targetInScreenY) = self.grid2Display((targetInGridX,targetInGridY))
        
 #       (loadingInGridX,loadingInGridY) = orderedTargetQueue.popitem(last=True)[0]
 #       for loadingCount in range(0,self.gridWidth*self.gridHeight):#if target and loading area too close, choose another loading area
 #           if(abs(loadingInGridX-targetInGridX)+abs(targetInGridY-loadingInGridY)<MINIMUM_TARGET_LOADING_DISTANCE):
 #               (loadingInGridX,loadingInGridY) = orderedTargetQueue.popitem(last=True)[0]
 #       (loadingInScreenX,loadingInScreenY) = self.grid2Display((loadingInGridX,loadingInGridY))
        
 #       self.setTarget(Target(Point(targetInScreenX,targetInScreenY),self.target.r))
 #       self.setLoadingArea(LoadingArea(Point(loadingInScreenX,loadingInScreenY),self.loadingArea.r))

class LoadingArea:
    def __init__(self, c, r):
        self.c = c
        self.r = r

class Target:
    def __init__(self, c, r):
        self.c = c
        self.r = r