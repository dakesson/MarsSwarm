#This script works in two modes which is switched with field.simulation:
#(1)Standalone simulator to study robot behavious using definitions from robot.py
#(2)Practise mode, position of robots are read from position.html, path for each robot is calculated and uploaded. 
#There are three resolution used in the script:
#(1)Actual resolution, used in position.html, range [0,3000] for x and [0,2400] for y, translated to screen resolution with field.actualFieldSize2DisplayField
#(2)Screen resolution, used for display in pygame, range [0,500] for x and [0,400] for y, translated to grid resolution with gridSize
#(3)Grid resolution, used for path searching, range [0,50] for x and [0,40] for y

import pygame, sys, time, random
from pygame.locals import *
from robot import Robot
from field import *
from path import *
from urllib.request import urlopen
import json


#test 
# set up pygame
pygame.init()

# set up the window
WINDOWWIDTH = 1002
WINDOWHEIGHT = 400
windowSurface = pygame.display.set_mode((WINDOWWIDTH, WINDOWHEIGHT), 0, 32)
pygame.display.set_caption('Simulator')
FIELDWIDTH = 500
FIELDHEIGHT = 400
ROBOTSMAXID = 20#maximum Robot ID, tuio could misread ID patttern and give crazy ID number
KINECT_IP = "http://192.168.0.101:8000"
TUIO_IP = "http://192.168.0.105:8000"
PATH_OUTPUT_FILE = 'apath.html'

# set up the colors

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY= (100,100,100)
LIGHTBLUE = Color(115,196,232,200)
robotColor = [(8,133,161),(187,86,149),(231,199,31),(175,54,60),(70,148,73),(56,61,150),(224,163,46),(157,188,64),
              (94,60,108),(193,90,99),(80,91,166),(214,126,44),(103,189,170),(133,128,177),(87,108,67),(98,122,157)]
#set target and loading area

gridSize = 10
displayGridSize = 2
field = Field(FIELDWIDTH,FIELDHEIGHT,gridSize)
field.setTarget(Target(Point(220,80),5))
field.setLoadingArea(LoadingArea(Point(200,399),5))

field.simulation = True
# set up the block data structure
robots = []
if(field.simulation):#if in simulation mode, load elevation from image
    field.weightedGrid.getElevationFromImage("elevation_test.png",field)
    field.targetGridElevation.getElevationFromImage("target_test.png",field)
    field.calculateTargetQueue()#update elevation differences
    for robotCount in range(0,2):
        robots.append(Robot(robotCount,Point(400,robotCount*100+100),Vector(2*random.random()-1,2*random.random()-1),field,robotColor[robotCount]))

# run the game loop
while True:
    # check for the QUIT event
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
    
    # draw the black background onto the surface
    windowSurface.fill(WHITE)
    
    #draw window spliter
    pygame.draw.line(windowSurface, BLACK, (FIELDWIDTH,0),(FIELDWIDTH,FIELDHEIGHT),2)
    #---------------------------------------------------------------------#
    #---------------------------------------------------------------------#
    if(not field.simulation):
        #######################################################################
        #update global elevaltion stored in field.weightedGrid.elevation
        #link = "http://192.168.0.101:8000" # for later over wifi
        try:
            previousElevationData = elevationData
        except:
            previousElevationData = {}
        try:
            #link = "http://192.168.0.101:8000"  # local machine ip

            filename = "/dmap.html" # json file saved as html with kinect data
            
            html = urlopen(KINECT_IP + filename)
            myfile = html.read().decode('utf-8')  # read file

            elevationData = json.loads(myfile)  # parse to json structure
        except:
            elevationData = previousElevationData
            print("Elevation html reading failed.")
        field.weightedGrid.getElevationJson(field,robots,elevationData)
        ##########################################################################
        #######################################################################
        #add robots
        try:
            previousPositionData = positionData
        except:
            previousPositionData = {}
        
        try:
            #link = "http://192.168.0.105:8000"  # local machine ip

            positionFile = "/positions.html" # json file saved as html with kinect data

            html = urlopen(TUIO_IP + positionFile)
            myfile = html.read().decode('utf-8')  # read file

            positionData = json.loads(myfile)
        except:
            positionData = previousPositionData
            print("Position html reading failed")
        for key in positionData.keys():
            if(any(str(x.ID) == key.split("RAM")[1] for x in robots) is False):
                if(int(key.split("RAM")[1])>ROBOTSMAXID):
                    continue
                #print(key.split("RAM"))
                robotCount = int(key.split("RAM")[1])
                robots.append(Robot(robotCount,Point(positionData[key]["pos"][0]/field.actualFieldSize2DisplayField,positionData[key]["pos"][1]/field.actualFieldSize2DisplayField),Vector(positionData[key]["vec"][0],positionData[key]["vec"][1]).unitize(),field,robotColor[robotCount]))
        
        #######################################################################	
        #draw grid with their elevation
        #=======================================================================
        # for h in range(field.gridWidth):
        #     for v in range(field.gridHeight):
        #         height2Color = int(field.weightedGrid.elevation[(h,v)]*255/((field.distanceMax-field.distanceMin)/field.distanceStep))
        #         pygame.draw.rect(windowSurface, (height2Color,height2Color,height2Color), Rect(h*gridSize,v*gridSize,gridSize,gridSize))
        #=======================================================================
    #---------------------------------------------------------------------#
    #---------------------------------------------------------------------#
    else:#in simulation mode, the elevation map is loaded from image
        positionData = {}
            #===================================================================
            # for h in range(field.gridWidth):
            #     for v in range(field.gridHeight):
            #         height2Color = int(field.weightedGrid.elevation[(h,v)])
            #         pygame.draw.rect(windowSurface, (height2Color,height2Color,height2Color), Rect(h*gridSize,v*gridSize,gridSize,gridSize))
            #===================================================================
    for h in range(field.gridWidth):
            for v in range(field.gridHeight):
                height2Color = int(field.weightedGrid.elevation[(h,v)]*255/field.maxElevation)
                pygame.draw.rect(windowSurface, (height2Color,height2Color,height2Color), Rect(h*gridSize,v*gridSize,gridSize,gridSize))
    #map background    
    #img=pygame.image.load("landscape.png") 
    #windowSurface.blit(img,(0,0))
        
    #draw target area
    pygame.draw.circle(windowSurface, (255,127,39,0), (field.target.c.X,field.target.c.Y),field.target.r,1)
    #draw loading area
    pygame.draw.circle(windowSurface, (125,190,255,0), (field.loadingArea.c.X,field.loadingArea.c.Y),field.loadingArea.r,1)
    # draw the block onto the surface
    
    #######################################################################
    #Update robots status, robots' behavious defined in robot.py
    for b in robots:        
        b.update(robots,positionData)
        b.display(windowSurface)
    #######################################################################
    #export robot path to server
    robotPathList = []
    #robotPath = {}
    for b in robots:
        robotPath = {}
        #robotPath['RAM'+str(b.ID)] = b.pathInWorld
        
        if(b.isRobotPaused):
            robotPath["pause"] = True
        else:
            robotPath["pause"] = False
        if(b.isSandloaded):
            robotPath["task"] = "dep"
        else:
            robotPath["task"] = "dig"
        robotPath["id"] = b.ID
        if not b.pathInWorld:
            robotPath["path"] = []
        else:
            robotPath["path"] = b.pathInWorld[0]
            
        robotPathList.append(robotPath) 
    with open(PATH_OUTPUT_FILE, 'w') as outfile:
        json.dump(robotPathList, outfile)
	##########################################################################
    #draw horizontal lines for each robot's path thumbnail
    for h in range(int(math.floor(FIELDWIDTH/field.gridHeight))):
        pygame.draw.line(windowSurface, BLACK, (FIELDWIDTH,h*field.gridHeight*displayGridSize),(WINDOWWIDTH,h*field.gridHeight*displayGridSize),1)
    #draw vertical lines
    for v in range(int(math.floor(FIELDWIDTH/field.gridWidth))):
        pygame.draw.line(windowSurface, BLACK, (v*field.gridWidth*displayGridSize+FIELDWIDTH,0),(v*field.gridWidth*displayGridSize+FIELDWIDTH,800),2)
    ##########################################################################
    # draw the window onto the screen
    pygame.display.update()
    time.sleep(0.0001)
