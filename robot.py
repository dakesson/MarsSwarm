#!/usr/bin/python

import pygame, sys, time, math, random
from pygame.locals import *
from field import *
from path import *
import pygame.gfxdraw


class Robot:
    def __init__(self, ID, point,direction,field,color):
        self.ID = ID#unique ID
        self.position = point#robot position in screen resolution
        self.field  = field
        self.positionInGrid = (int(self.position.X/self.field.gridSize),int(self.position.Y/self.field.gridSize))
        self.direction = direction.unitize()
        self.isSandloaded = False#boolean whether sand is loaded
        self.targetFound = False
        #self.loadingBayFound = False
        self.currentOrientation = Vector(0,1)#
        self.neighborGridElevation = {}
        
        self.targetPathGridCount = 0#index of item in self.path
        self.path = []#list of grid points, path to go
        self.pathInWorld = []#list of grid points in actual resolution       
        self.targetInGrid = field.loadingAreaInGrid#looking for loading area first
        self.weightedGrid = GridWithWeights(self.field.gridWidth,self.field.gridHeight)   #each robot has its own map of elevation and grid has been explored     
        self.exploredGrid = {}#record explored grid, for example:{[0,0]:1,[0,1]:1}
        self.doesPathRequireUpdate = False
        self.isRobotPaused = False
        self.color = color
        #initilize robot's elevation map
        for k,v in field.weightedGrid.elevation.items():
            self.weightedGrid.elevation[k]=0
        self.searchPath(self.targetInGrid)
     
    def update(self,robots,positionData):#update robot status, this will update robot's position, orientation and path        
        self.targetFound = False

        v1 = rule1(self,robots,positionData)
        v2 = 

    def rule1(self,robots):
        pcj = Vector(0,0)
        for robot in robots:
            if robot != self:
                pcj.add(robot.position)
        pcj = pcj / (len(robots)-1)
        return (pcj - self.position)/100

    def rule2(self,robots):
        c = Vector(0,0,0)

        for robot in robots:
            if robot != self:
                if abs(self.position.distance())


    
    def checkSensor(self,robots,positionData):#check robot virtual sensors: based on the elevation of area just found, determine whether path need to be updated
        if(not self.field.simulation):#Not in simulation mode
            ROBOT_SHIFT_TOLERANCE = 10
            #######################################################################
            #get position of robots from server
            previousPosition = self.position
            for key in positionData.keys():
                if (self.ID == positionData[key]["id"]):#IS current robot
                    self.position = Point(positionData[key]["pos"][0]/self.field.actualFieldSize2DisplayField,(self.field.actualFieldSize2DisplayField*self.field.gridHeight*self.field.gridSize-positionData[key]["pos"][1])/self.field.actualFieldSize2DisplayField);
                    self.direction = Vector(positionData[key]["vec"][0],positionData[key]["vec"][1]).unitize();
                    self.positionInGrid = (int(self.position.X/self.field.gridSize),int(self.position.Y/self.field.gridSize))
        #######################################################################
            self.doesPathRequireUpdate = self.updateExploredArea()
            if(previousPosition.distance(self.position)>ROBOT_SHIFT_TOLERANCE):
                self.doesPathRequireUpdate = True
        
        else:#simulation mode            
            self.doesPathRequireUpdate = self.updateExploredArea()
        #make decision whether it need to dig or deposit, this is based on there factors:
        #(1)if robot is in a certain range to the deposit location/dig location
        #(2)if the difference from current elevation to target elevation is big enough
        #(3)if in the sensing range of the robot, there is another position has bigger elevation differences
         
        #=======================================================================
        # if(self.position.distance(self.field.target.c)<self.field.target.r or self.position.distance(self.field.loadingArea.c)<self.field.loadingArea.r): 
        #     neighbors = list(self.weightedGrid.neighbors(self.positionInGrid,3))#this is the same radius as in explored area
        #     currentElevationDifference = abs(self.weightedGrid.elevation[self.positionInGrid]-self.field.targetGridElevation.elevation[self.positionInGrid])
        #     #print("Robot"+str(self.ID)+" elevation difference is "+str(self.positionInGrid)+" "+str(currentElevationDifference))
        #     if(currentElevationDifference>0):
        #         self.targetFound = True
        #=======================================================================
            #===================================================================
            # for positionInNeighbors in neighbors:
            #     #print(positionInNeighbors)
            #     (positionInScreenX,positionInScreenY) = self.field.grid2Display(positionInNeighbors)
            #     currentPosition = Point(positionInScreenX,positionInScreenY)
            #     #print("Robot"+str(self.ID)+" elevation difference is "+str(positionInNeighbors)+" "+str(abs(self.weightedGrid.elevation[positionInNeighbors]-self.field.targetGridElevation.elevation[positionInNeighbors])))
            #     if(currentPosition.distance(self.field.target.c)<self.field.target.r or currentPosition.distance(self.field.loadingArea.c)<self.field.loadingArea.r):                    
            #         thisElevationDifference = abs(self.weightedGrid.elevation[positionInNeighbors]-self.field.targetGridElevation.elevation[positionInNeighbors])
            #         if(thisElevationDifference>currentElevationDifference):
            #             currentElevationDifference = thisElevationDifference
            #             self.targetFound = False
            #             self.targetInGrid = positionInNeighbors
            #             self.doesPathRequireUpdate = True
            #             print("update path from neighbors")
            #===================================================================
                        
            #print("target is "+str(positionInNeighbors))
        ROBOT_CLOEST_DISTANCE = 80#equals to 80*6=480 mm in actual resolution
        COMMUNICATION_DISTANCE = 80#equals to 80*6=480 mm in actual resolution
        self.weightedGrid.walls = []#clear all walls
        for robot in robots:#check robots in the robots list
            if robot.ID == self.ID:
                continue
            elif(robot.position.distance(self.position)<ROBOT_CLOEST_DISTANCE):#if two robots are close to each other, add walls to weightedGrid to avoid collision
                #self.isRobotPaused = True
                self.weightedGrid.walls.extend(list(self.weightedGrid.neighbors(robot.positionInGrid,3)))#Need to check that target could also be in the wall

                #print("walls at"+str(self.weightedGrid.walls))
                # if(self.positionInGrid[0] is not robot.positionInGrid[0]):
                    # print(str(self.positionInGrid[1])+" "+str(robot.positionInGrid[1])+" y")
                    # print(str(self.positionInGrid[0])+" "+str(robot.positionInGrid[0])+" x")
                    # for y in range(min(self.positionInGrid[1],robot.positionInGrid[1]),max(self.positionInGrid[1],robot.positionInGrid[1])+1):
                       # self.weightedGrid.walls.append((min(self.positionInGrid[0],robot.positionInGrid[0])+1,y))
                       # print("y "+str((min(self.positionInGrid[0],robot.positionInGrid[0])+1,y)))
                #print("Robot"+str(robot.ID)+" and Robot"+str(self.ID)+" is close to each other.")
                self.doesPathRequireUpdate = True
            #else:#if two robots are not close to each other, remove walls
                #self.weightedGrid.walls = []
            #if a robot is found within communication range, update each other's map
            if((robot.position.distance(self.position)<COMMUNICATION_DISTANCE) ):#and (cmp(self.weightedGrid.elevation,robot.weightedGrid.elevation) != 0)
                try:
                    self.weightedGrid.elevation.update(robot.weightedGrid.elevation)
                    self.exploredGrid.update(robot.exploredGrid)
                    robot.weightedGrid.elevation.update(self.weightedGrid.elevation)
                    robot.exploredGrid.update(self.exploredGrid)
                    self.doesPathRequireUpdate = True
                    #print("Robot"+str(robot.ID)+" and Robot"+str(self.ID)+" update their maps.")
                except:
                    print("Robot"+str(robot.ID)+" and Robot"+str(self.ID)+" error updateing maps.")
                
                
    
    def getPath(self,robots):#update robot's path
        #Need to update target?
        if(self.isSandloaded == False):#if sand is not loaded, use loading area as target
            if(self.targetInGrid is not self.field.loadingAreaInGrid):
            #if(self.field.gridDistance2ScreenDistance(self.targetInGrid,self.field.loadingAreaInGrid)>self.field.loadingArea.r):
                self.doesPathRequireUpdate = True
                self.targetInGrid = self.field.loadingAreaInGrid
                #print("target changed")
        else:#if sand is loaded, if target is not target area, change target
            if(self.targetInGrid is not self.field.targetInGrid):
            #if(self.field.gridDistance2ScreenDistance(self.targetInGrid,self.field.targetInGrid)>self.field.target.r):                
                self.doesPathRequireUpdate = True
                self.targetInGrid = self.field.targetInGrid  
                #print("target changed")          
        
        if(self.doesPathRequireUpdate):#if found new terrain, or changed target, update path
            self.searchPath(self.targetInGrid)
            self.targetPathGridCount = 0
            #print("Robot"+str(self.ID)+" recalculated path.")
        
        #foundTarget = False
        
        #if(self.field.simulation):# In simulation mode, move the robot based on determined path
        if(self.isRobotPaused is True):
            self.pauseRobot()
            self.isRobotPaused = False
            #print("Robot"+str(self.ID)+" is waiting")
        else:            
            #following the current path    
            self.targetFound = self.followPath()
            #print("Robot"+str(self.ID)+"follow path")
        #if target is found and sand is not loaded, loadsand
        if(self.targetFound and (not self.isSandloaded)):
            self.isSandloaded = True
            if(self.field.simulation):
                self.digDip(2, 5, True)
            print("Robot"+str(self.ID)+" loaded sand.")
            self.field.calculateTargetQueue()#update elevation differences
        elif(self.targetFound and (self.isSandloaded)):
            self.isSandloaded = False
            if(self.field.simulation):
                self.digDip(2, 5, False)                    
            print("Robot"+str(self.ID)+" unloaded sand.")
            self.field.calculateTargetQueue()#update elevation differences

    def followPath(self):#follow the current path, if reached the target, return true, else, return false
        FOUND_TARGET_DISTANCE = 5
        if(not self.path):#if path is not available
            #self.pauseRobot()
            #if reached target
            if(self.position.distance(self.field.target.c)<FOUND_TARGET_DISTANCE or self.position.distance(self.field.loadingArea.c)<FOUND_TARGET_DISTANCE):
                return True
            else:
                return False
        elif((self.position.distance(self.path[len(self.path)-1])<FOUND_TARGET_DISTANCE)):
            #self.pauseRobot()
            return True
        else:
            if(self.position.distance(self.path[self.targetPathGridCount])<FOUND_TARGET_DISTANCE):#if reached grid point in path[]
                self.targetPathGridCount=self.targetPathGridCount+1#aiming for the next grid point in path[]
            if(self.field.simulation):
                self.direction = Vector(self.path[self.targetPathGridCount].X-self.position.X,self.path[self.targetPathGridCount].Y-self.position.Y).unitize()
                self.position = Point(self.position.X+self.direction.X,self.position.Y+self.direction.Y)
                self.positionInGrid = (int(self.position.X/self.field.gridSize),int(self.position.Y/self.field.gridSize))
            return False
                
    def pauseRobot(self):#pause robot in simulaiton mode
        self.position = self.position
        self.direction = self.direction        
    
    def searchPath(self,target):#seach path using a* algorithm based on the robot's elevation map and target location
        came_from, cost_so_far = a_star_search(self.weightedGrid,(int(self.position.X/self.field.gridSize), int(self.position.Y/self.field.gridSize)), target)
        path = reconstruct_path(came_from, start=(int(self.position.X/self.field.gridSize), int(self.position.Y/self.field.gridSize)), goal=target)        
        #self.pathInGrid = path
        if(path is not None):
            if(len(path)>1):
                path.pop(0)#remove current grid at index 0
            tempWorldPath = []
            del self.path[:]        
            for position in path:
                tempWorldPath.append([self.field.actualFieldSize2DisplayField*position[0]*self.field.gridSize+self.field.gridSize/2,self.field.actualFieldSize2DisplayField*self.field.gridHeight*self.field.gridSize-self.field.actualFieldSize2DisplayField*position[1]*self.field.gridSize+self.field.gridSize/2])
                self.path.append(Point(position[0]*self.field.gridSize+self.field.gridSize/2,position[1]*self.field.gridSize+self.field.gridSize/2))  
            self.pathInWorld = tempWorldPath#path is translated to actual resolution
        else:#if path is not available
            self.isRobotPaused = True
            self.path = []
            self.pathInWorld = []
            #print("Robot"+str(self.ID)+" can not find path.")
            
        
    def updateExploredArea(self):# return true or false on whether it is needed to update path
        ELEVATION_DIFFERENCE_THRESHOLD = 5
        #get elevation and grid for the area just explored (area around the robot)
        neighborGrid, neighborGridElevation = self.field.weightedGrid.neighborElevation(self.positionInGrid)
        self.neighborGridElevation = neighborGridElevation
        
        #get newly found grid
        #currentList = set(self.exploredGrid)
        #neighborList = set(neighborGrid)
        #newList = neighborList-currentList
        #self.exploredGrid = self.exploredGrid + list(newList)        
        foundGrid = {}
        for k, v in neighborGrid.items():
            #if (k not in self.exploredGrid) or (neighborGridElevation[k] !=self.weightedGrid.elevation[k]):#original: if neighbour not been explored before, or newly explored elevation not the same as previous value in self.weightedGrid 
            #print(str(neighborGridElevation[k])+" "+str(self.weightedGrid.elevation[k]))
            if (neighborGridElevation[k] !=self.weightedGrid.elevation[k]):#It does not matter area has need explored before, as long as newly explored elevation not the same as previous value in self.weightedGrid, update elevation
                foundGrid[k]=1
      
        #update explored grid and explored weighted grid
        self.weightedGrid.elevation.update(neighborGridElevation)
        self.exploredGrid.update(neighborGrid)        
                
        for k,v in foundGrid.items():
            #if found new grid with elevation different from current larger than threshold
            if(abs(self.weightedGrid.elevation[k]-self.field.weightedGrid.elevation[self.positionInGrid])>ELEVATION_DIFFERENCE_THRESHOLD):
                return True
            else:
                return False
        #if there is no terrain found or changed, return false(does not change path)
        return False
        
        
    def digDip(self,radius,depth,digOrDip):#digOrDip, true for dig and false for diposit
        for gridPosition in list(self.weightedGrid.neighbors(self.positionInGrid,radius)):
            if(digOrDip):
                self.field.weightedGrid.elevation[gridPosition] = max(0,self.weightedGrid.elevation[gridPosition]-depth)
            else:
                self.field.weightedGrid.elevation[gridPosition] = min(self.field.maxElevation,self.weightedGrid.elevation[gridPosition]+depth)
                
   #this function is for simulator only, the current status of the robot is displayed with pygame
    def display(self,windowSurface):
        BLACK = (0, 0, 0)
        WHITE = (255, 255, 255)
        GRAY= (100,100,100)
        rotation = 0
        DISPLAY_GRIDSIZE = 2
        surf =  pygame.Surface((21, 31))
        surf.fill((255, 255, 255))
        if((self.currentOrientation.X==self.direction.X)&(self.currentOrientation.Y==self.direction.Y)):
            rotation=0
        else:
            rotation = 180*self.currentOrientation.angle(self.direction)/3.14159
        VECTOR_LENGTH = 15
        CIRCLE_RADIUS = 13
        pygame.draw.circle(windowSurface, self.color, (int(self.position.X),int(self.position.Y)),int(CIRCLE_RADIUS*2),2)
        pygame.draw.line(windowSurface, self.color,(self.position.X+self.direction.X*CIRCLE_RADIUS,self.position.Y+self.direction.Y*CIRCLE_RADIUS),(self.position.X+self.direction.X*(CIRCLE_RADIUS+VECTOR_LENGTH),self.position.Y+self.direction.Y*(CIRCLE_RADIUS+VECTOR_LENGTH)),2)
       
        #print path of the robot
        printPath = []
        for pathCount in range(0, len(self.path)):
            printPath.append(self.path[pathCount].get())
        if(len(printPath)>2):
#             try:
            pygame.gfxdraw.bezier(windowSurface,printPath,30,WHITE)
#             except Exception as ex:
#                 template = "An exception of type {0} occured. Arguments:\n{1!r}"
#                 message = template.format(type(ex).__name__, ex.args)
#                 print (message)
#                 print("Robot"+str(self.ID)+" error display path.")
        #display explored area
        for h in range(self.field.gridWidth):
            for v in range(self.field.gridHeight):
                if((h,v) in self.weightedGrid.walls):
                    pygame.draw.rect(windowSurface,WHITE,Rect(self.field.width+h*DISPLAY_GRIDSIZE+self.ID%4*self.field.gridWidth*DISPLAY_GRIDSIZE,v*DISPLAY_GRIDSIZE+self.ID//4*self.field.gridHeight*DISPLAY_GRIDSIZE,DISPLAY_GRIDSIZE,DISPLAY_GRIDSIZE))
                elif((h,v)==self.positionInGrid):
                    pygame.draw.rect(windowSurface,BLACK,Rect(self.field.width+h*DISPLAY_GRIDSIZE+self.ID%4*self.field.gridWidth*DISPLAY_GRIDSIZE,v*DISPLAY_GRIDSIZE+self.ID//4*self.field.gridHeight*DISPLAY_GRIDSIZE,DISPLAY_GRIDSIZE,DISPLAY_GRIDSIZE))
                elif((h,v) in self.exploredGrid):
                    #print(str(self.ID))
                    pygame.draw.rect(windowSurface,self.color,Rect(self.field.width+h*DISPLAY_GRIDSIZE+self.ID%4*self.field.gridWidth*DISPLAY_GRIDSIZE,v*DISPLAY_GRIDSIZE+self.ID//4*self.field.gridHeight*DISPLAY_GRIDSIZE,DISPLAY_GRIDSIZE,DISPLAY_GRIDSIZE))
                else:
                    pygame.draw.rect(windowSurface,GRAY,Rect(self.field.width+h*DISPLAY_GRIDSIZE+self.ID%4*self.field.gridWidth*DISPLAY_GRIDSIZE,v*DISPLAY_GRIDSIZE+self.ID//4*self.field.gridHeight*DISPLAY_GRIDSIZE,DISPLAY_GRIDSIZE,DISPLAY_GRIDSIZE))
                
                
     #   for v in range(FIELDHEIGHT/gridSize):
            #if (pathCount!=len(self.path)-1):
             #   pygame.draw.line(windowSurface, GRAY,(self.path[pathCount].X+field.width,self.path[pathCount].Y),(self.path[pathCount+1].X+field.width,self.path[pathCount+1].Y),2)
        #pygame.draw.rect(surf, BLACK, pygame.Rect(0,0, 20, 30),2)
        #where = self.position.X, self.position.Y        
        #blittedRect = windowSurface.blit(surf, where)
        #windowSurface.blit(windowSurface, where, where)
        
        #rotatedSurf =  pygame.transform.rotate(surf, rotation)
        #rotRect = rotatedSurf.get_rect()
        #rotRect.center = blittedRect.center
        
        #windowSurface.blit(rotatedSurf, rotRect)
        #self.currentOrientation = self.direction
