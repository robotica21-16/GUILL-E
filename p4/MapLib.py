#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
import time
import os
import math

class Map2D:
    def __init__(self, map_description_file):
        """
        Load and initialize map from file. \

        map_description_file: path to a text file containing map description in the standard format. \
        Example for a 3x3 grid map, with (squared) cells of 400mm side length called mapa0. \
        All free space, i.e., all connections between cells are open, except those on the limits of the map.
        For more details on the format, see class documentation.

        mapa0.txt content:
        3 3 400
        0 0 0 0 0 0 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 0 0 0 0 0 0

        """
        # params to visualize
        self.mapLineStyle='r-'
        self.costValueStyle='g*'
        self.verbose = True
        # set to False to stop displaying plots interactively (and maybe just save the screenshots)
        # self.verbose = False
        self.current_ax = None

        # variables about map params
        self.sizeX=0
        self.sizeY=0
        self.sizeCell=0

        self.connectionMatrix = None
        self.costMatrix =  None
        self.currentPath =  None

        #self.endx
        #self.endy

        if self._loadMap(map_description_file):
            print("Map %s loaded ok" % map_description_file)
        else:
            print("Map %s NOT loaded" % map_description_file)


    # from python docs: https://docs.python.org/3/tutorial/classes.html#private-variables
    # “Private” instance variables that cannot be accessed except from inside an object don’t exist in Python.
    # However, there is a convention that is followed by most Python code: a name prefixed with an underscore \
    # (e.g. _spam) should be treated as a non-public part of the API (whether it is a function, a method or a data member).

    # ############################################################
    # private methods
    # ############################################################
    def _initConnections(self, init_value=0):
        """
        to initialize the matrix, we set all connections to be closed.
        When the file with the description is loaded, it will "open" (set to 1) the corresponding ones.
        """
        self.connectionMatrix = np.ones( (2*self.sizeX+1, 2*self.sizeY+1) ) * init_value

    def _initCostMatrix(self, init_value=-2):
        """
        to initialize the matrix, we set all connections to be closed.
        When the file with the description is loaded, it will "open" (set to 1) the corresponding ones.
        """
        self.costMatrix = np.ones( (self.sizeX, self.sizeY) ) * init_value

        # Example costMatrix (filled manually!) for Map1
        # if we plan to go from 0,0 to 2,0
        # self.costMatrix[2,0] = 0
        # self.costMatrix[1,0] = 1
        # self.costMatrix[1,1] = 2
        # self.costMatrix[1,2] = 3
        # self.costMatrix[0,2] = 4
        # self.costMatrix[2,2] = 4
        # self.costMatrix[0,1] = 5
        # self.costMatrix[2,1] = 5
        # self.costMatrix[0,0] = 6



    def _loadMap(self, mapFileName):
        """
        Load map from a txt file (mapFileName) to fill the map params and connectionMatrix. \
        NOTES: \
        \t connectionMatrix is a numpy array \
        \t Function will return False if something went wrong loading the map file.
        """
        try:
            # FILL GLOBAL VARIABLES dimX dimY cellSize
            loadingOk=False
            mapF = open(mapFileName, "r")

            # 1. special case for first line. initialize dimX dimY cellSize
            header = mapF.readline() #next()
            tmp = header.split() # any whitespace string is a separator and empty strings are removed from the result
            if self.verbose:
                print("Header line: %s " % header)
            parsed_header = [int(c) for c in tmp]
            # expected to have three numbers: sizeX sizeY sizeCell_in_mm
            if len(parsed_header)==3:
                self.sizeX, self.sizeY, self.sizeCell = parsed_header
            else:
                print("Wrong header in map file: %s" % header)
                return False

            # 2.init connectionMatrix and costMatrix
            self._initConnections()
            self._initCostMatrix()

            # 3. load rest of the map connection lines information
            for indx, line in enumerate(mapF):
                # we start loading from the file the "top" row of the map
                current_row = (self.connectionMatrix.shape[1]-1) - indx
                # Split numbers in the line. Any whitespace string is a separator and empty strings are removed from the result
                tmp = line.split()
                if self.verbose:
                    print("Line for map row %d: %s " % (current_row, line))
                parsed_line = [int(c) for c in tmp]

                if len(parsed_line) == self.connectionMatrix.shape[0] and indx < self.connectionMatrix.shape[1]:
                    self.connectionMatrix[:, current_row] = parsed_line
                elif len(parsed_line): # don't give errors because of empty lines
                    print("Wrong connectionMatrix (%s) row data: %s" % (self.connectionMatrix.shape(), line) )
                    return False
            mapF.close()
            loadingOk = True
        except Exception as e:
            print("ERROR:", e.__doc__)
            print(e)
            #raise
            loadingOk = False

        return loadingOk

    def _cell2connCoord(self, cellX, cellY, numNeigh):
        """
        Input:
            cellX, cellY: cell coordinates (cellX, cellY) in the map grid
            numNeigh: index of one of the cell 8-neighbours

        Output:
            (connX,connY): 2D coordinates (in the connectionMatrix!!) \
            of the connection of the input cell to the input neighbour
        """
        connX=2*cellX+1
        connY=2*cellY+1
        p = [connX, connY]

        result = {
            0: lambda p: [ p[0],    p[1]+1],
            1: lambda p: [ p[0]+1,  p[1]+1],
            2: lambda p: [ p[0]+1,  p[1]],
            3: lambda p: [ p[0]+1,  p[1]-1],
            4: lambda p: [ p[0],    p[1]-1],
            5: lambda p: [ p[0]-1,  p[1]-1],
            6: lambda p: [ p[0]-1,  p[1]],
            7: lambda p: [ p[0]-1,  p[1]+1],
        }

        return result[numNeigh](p)

    def _pos2cell(self, x_mm, y_mm):
        """ Convert from robot odometry coordinates (in mm) to cell coordinates """
        # make sure we discretize the result to the closest lower integer value
        x_cell = int(np.floor(x_mm/self.sizeCell))
        y_cell = int(np.floor(y_mm/self.sizeCell))
        return [x_cell, y_cell]
    #
    # def _pos2cell_m(self, x_m, y_m):
    #


    # ############################################################
    # public methods
    # ############################################################
    def setConnection(self, cellX, cellY, numNeigh):
        """
        open a connection, i.e., we can go straight from cellX,cellY to its neighbour number numNeigh
        """
        # from coordinates in the grid of cells to coordinates in the connection matrix
        [connX, connY] = self._cell2connCoord(cellX, cellY, numNeigh)
        self.connectionMatrix[connX, connY]=1 # True

    def deleteConnection(self, cellX, cellY, numNeigh):
        """
        close a connection, i.e., we can NOT go straight from cellX,cellY to its neighbour number numNeigh
        """
        # from coordinates in the grid of cells to coordinates in the connection matrix
        [connX, connY] = self._cell2connCoord(cellX, cellY, numNeigh)
        self.connectionMatrix[connX, connY] = 0 # False

    def isConnectedNumber(self, cellX, cellY, numNeigh):
        """
        returns True if the connnection from cell (x,y) to its neighbour number numNeigh is open.

        The neighbour indexing is considered as follows
        (8-neighbours from cell x,y numbered clock-wise):

        7     0       1
        6   (x,y)     2
        5     4       3

        """
        [connX, connY] = self._cell2connCoord(cellX, cellY, numNeigh)

        return self.connectionMatrix[connX, connY]

    def isConnected(self, cellX, cellY, numNeigh):
        """
        returns True if the connnection from cell (x,y) to its neighbour number numNeigh is open.

        The neighbour indexing is considered as follows
        (8-neighbours from cell x,y numbered clock-wise):

        7     0       1
        6   (x,y)     2
        5     4       3

        """
        n = self.isConnectedNumber(cellX, cellY, numNeigh)
        return n>0.5

    # aux functions to display (or save image) with robot and map stuff
    def _drawGrid(self):
        """
        aux function to create a grid with map lines
        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        plt.rc('grid', linestyle="--", color='gray')
        plt.grid(True)
        plt.tight_layout()

        x_t = range(0, (self.sizeX+1)*400, 400)
        y_t = range(0, (self.sizeY+1)*400, 400)
        x_labels = [str(n) for n in x_t]
        y_labels = [str(n) for n in y_t]
        plt.xticks(x_t, x_labels)
        plt.yticks(y_t, y_labels)

        # Main rectangle
        X = np.array([0, self.sizeX, self.sizeX, 0,          0]) * self.sizeCell
        Y = np.array([0, 0,          self.sizeY, self.sizeY, 0]) * self.sizeCell
        self.current_ax.plot(X, Y, self.mapLineStyle)

        # "vertical" walls
        for i in range(2, 2*self.sizeX, 2):
            for j in range(1, 2*self.sizeY, 2):
                if not self.connectionMatrix[i,j]:
                    # paint "right" wall from cell (i-1)/2, (j-1)/2
                    cx= np.floor((i-1)/2)
                    cy= np.floor((j-1)/2)
                    X = np.array([cx+1, cx+1]) * self.sizeCell
                    Y = np.array([cy, cy+1]) * self.sizeCell
                    self.current_ax.plot(X, Y, self.mapLineStyle)

        # "horizontal" walls
        for j in range(2, 2*self.sizeY, 2):
            for i in range(1, 2*self.sizeX, 2):
                if not self.connectionMatrix[i,j]:
                    # paint "top" wall from cell (i-1)/2, (j-1)/2
                    cx=np.floor((i-1)/2)
                    cy=np.floor((j-1)/2)
                    X = np.array([cx, cx+1]) * self.sizeCell
                    Y = np.array([cy+1, cy+1]) * self.sizeCell
                    self.current_ax.plot(X, Y, self.mapLineStyle)
        plt.axis('equal')

        return True


    # aux functions to display the current CostMatrix on the map
    def _drawCostMatrix(self):
        """
        aux function to create a grid with map lines
        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        # "center" of each cell
        for i in range(0, self.sizeX):
            for j in range(0, self.sizeY):
                    cx= i*self.sizeCell + self.sizeCell/2.
                    cy= j*self.sizeCell + self.sizeCell/2.
                    X = np.array([cx])
                    Y = np.array([cy])
                    cost = self.costMatrix[i,j]
                    self.current_ax.text(X, Y, str(cost))


        plt.axis('equal')

        return True

    # Dibuja robot en location_eje con color (c) y tamano (p/g)
    def _drawRobot(self, loc_x_y_th=[0,0,0], robotPlotStyle='b', small=False):
        """
        UPDATES existing plot to include current robot position
        It expects an existing open figure (probably with the map already on it)

        loc_x_y_th is the position x,y and orientation in mm and radians of the main axis of the robot

        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        if small:
            largo, corto, descentre = [80, 50, 5]
        else:
            largo, corto, descentre = [160, 100, 10]

        trasera_dcha=np.array([-largo,-corto,1])
        trasera_izda=np.array([-largo,corto,1])
        delantera_dcha=np.array([largo,-corto,1])
        delantera_izda=np.array([largo,corto,1])
        frontal_robot=np.array([largo,0,1])

        tita=loc_x_y_th[2]
        Hwe=np.array([[np.cos(tita), -np.sin(tita), loc_x_y_th[0]],
                 [np.sin(tita), np.cos(tita), loc_x_y_th[1]],
                  [0,        0 ,        1]])

        Hec=np.array([[1,0,descentre],
                  [0,1,0],
                  [0,0,1]])

        extremos=np.array([trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
        robot=np.dot(Hwe, np.dot(Hec,np.transpose(extremos)))

        self.current_ax.plot(robot[0,:], robot[1,:], robotPlotStyle)

        return True

    def drawMapWithRobotLocations(self,
                                  robotPosVectors=[ [0,0,0], [600, 600, 3.14] ],
                                  saveSnapshot=True):
        """ Overloaded version of drawMap to include robot positions """
        return self.drawMap(robotPosVectors=robotPosVectors, saveSnapshot=saveSnapshot)


    def drawMap(self, robotPosVectors = None, saveSnapshot=False):
        """
        Generates a plot with currently loaded map status

        NOTE:
        if verbose, it displays the plot
        if saveSnapshot: saves a figure as mapstatus_currenttimestamp_FIGNUM.png
        """
        self.verbose=True
        #self.verbose=False

        # create a new figure and set it as current axis
        current_fig = plt.figure()
        self.current_ax = current_fig.add_subplot(111)

        self._drawGrid()

        # if flag is true, draw also current CostMatrix
        if self.verbose:
            self._drawCostMatrix()

        if robotPosVectors:
            for loc in robotPosVectors:
                print("Robot in pos: ", loc)
                self._drawRobot(loc_x_y_th=loc, robotPlotStyle='b--')
            # plot last robot position with solid green line
            self._drawRobot(loc_x_y_th=loc, robotPlotStyle='g-')

        if saveSnapshot:
            ts = str(time.time())
            snapshot_name = "mapstatus_"+ts+"_F"+str(current_fig.number)+".png"
            print("saving %s " % snapshot_name)
            plt.savefig(snapshot_name)

        if self.verbose:
            current_fig.set_visible(True)
            current_fig.show()
            print("Press ENTER in the plot window to continue ... ")
            current_fig.waitforbuttonpress()
        else:
            current_fig.set_visible(False)

        return current_fig


    def findPath(self, point_ini, point_end):
        """ overloaded call to planPath (x_ini,  y_ini, x_end, y_end) """
        return self.planPath(point_ini[0], point_ini[1],
                             point_end[0], point_end[1])

    # ############################################################
    # METHODS to IMPLEMENT in P4
    # ############################################################

    def neighbourCell(self, x, y, neighbour):
        if neighbour==0:
            return [x, y+1]
        elif neighbour==2:
            return [x+1, y]
        elif neighbour==4:
            return [x, y-1]
        elif neighbour==6:
            return [x-1, y]

    def printCostMatrix(self):
        """
        Prints the cost matrix in the same order as the maps (X right, Y up)
        """
        print(np.rot90(self.costMatrix))

    def hasValue(self, neighbour_cell, front):
        """
        returns True if the cell already has a value or is in the frontier
        """
        return neighbour_cell in front or self.costMatrix[neighbour_cell[0], neighbour_cell[1]] >= 0
        
    def cellExists(self, cell):
        x=cell[0]
        y=cell[1]
        return 0<=x<self.sizeX and 0<=y<self.sizeY

    def fillCostMatrix(self):
        """
        Fills the cost matrix with the costs of getting to goal=[x,y].
        If any cells are unreachable they keep cost -2
        """
        self._initCostMatrix()
        end = False
        cost = 0
        frontier = [self.goal]
        procesadas = 0
        
        while not end:
            newFront = []
            if not frontier: # is empty
                print('EMPTY FRONTIER')
                end = True
                self.drawMap(saveSnapshot=False)
                return False

            for cell in frontier:
                self.costMatrix[cell[0], cell[1]] = cost
                for i in range(4): # for each neighbour
                    neighbour = i*2
                    neighbour_cell = self.neighbourCell(cell[0], cell[1],neighbour)
                    if self.isConnected(cell[0], cell[1], neighbour) and self.cellExists(neighbour_cell): 
                        if not self.hasValue(neighbour_cell, newFront):
                            newFront += [neighbour_cell]
                procesadas+=1
                if procesadas >= self.costMatrix.size:
                    end=True
            frontier=newFront
            cost += 1
        self.printCostMatrix()
        return True




    def findPath(self, x_ini,  y_ini, x_end, y_end):
        """
        x_ini, y_ini, x_end, y_end: integer values that indicate \
            the x and y coordinates of the starting (ini) and ending (end) cell
        Finds the path between ini and end
        """
        self.goal = [x_end,y_end]
        if not self.fillCostMatrix():
            return False

        self.currentPath = []
        pathFound = False
        current_x=x_ini
        current_y=y_ini
        self.endx=x_end
        self.endy=y_end
        while not pathFound:
            x_min=0
            y_min=0
            min_cost=math.inf
            foundOne=False
            if(self.isConnected(current_x, current_y, 0) and self.costMatrix[current_x][current_y+1]<min_cost):
                x_min=current_x
                y_min=current_y+1
                min_cost=self.costMatrix[x_min][y_min]
            if(self.isConnected(current_x, current_y, 6) and self.costMatrix[current_x-1][current_y]<min_cost):
                x_min=current_x-1
                y_min=current_y
                min_cost=self.costMatrix[x_min][y_min]
            if(self.isConnected(current_x, current_y, 2) and self.costMatrix[current_x+1][current_y]<min_cost):
                x_min=current_x+1
                y_min=current_y
                min_cost=self.costMatrix[x_min][y_min]
            if(self.isConnected(current_x, current_y, 4) and self.costMatrix[current_x][current_y-1]<min_cost):
                x_min=current_x
                y_min=current_y-1
                min_cost=self.costMatrix[x_min][y_min]
            if math.isinf(min_cost):
                print("Blocked path in ", current_x, current_y)
                print("Current path:",self.currentPath )
                return False
            current_x=x_min
            current_y=y_min
            self.currentPath+=[[current_x, current_y]]
            if(current_x==x_end and current_y==y_end):
                pathFound=True
        return pathFound

    def thToNeighbour(self, th):
        th = int(4*th/math.pi)
        n = 0

        if th == 0:
            n = 2
        elif th == 1:
            n = 1
        elif th == 2:
            n = 0
        elif th == 3:
            n = -1


        n = int((math.pi + th) / math.pi * 8)+2
        return n

    def obstacleDetected(self,x_now, y_now, x_2, y_2):
        """
        Adds the obstacle between the cells to the map
        """
        x_milli = max(x_now*1000.0, 0)
        y_milli = max(y_now*1000.0, 0)
        x_now, y_now = self._pos2cell(x_milli, y_milli)
        endx,endy=self._pos2cell(x_2*1000.0, y_2*1000.0)
        neighbour = self.neighbourFromCells([x_now, y_now],[endx, endy])
        self.deleteConnection(x_now, y_now, neighbour)


    def replanPath(self, x, y, x_end=-1, y_end=-1):
        """
        replans the path from x, y to the new goal or the previous one if
        none is provided
        """
        x_milli = max(x*1000.0, 0)
        y_milli = max(y*1000.0, 0)
        inix,iniy=self._pos2cell(x_milli, y_milli)
        if(x_end == -1 or y_end == -1):
            return self.findPath(inix, iniy, self.endx, self.endy)
        else:
            inix,iniy=self._pos2cell(x_end*1000.0, y_end*1000.0)
            return self.findPath(inix, iniy, x_end, y_end)


    def neighbourFromCells(self, cell1, cell2):

        dif1 = cell2[0]-cell1[0]
        dif2 = cell2[1]-cell1[1]
        res=None
        if dif1==0 and dif2==1:
            res=0
        elif dif1==1 and dif2==1:
            res=1
        elif dif1==1 and dif2==0:
            res=2
        elif dif1==1 and dif2==-1:
            res=3
        elif dif1==0 and dif2==-1:
            res=4
        elif dif1==-1 and dif2==-1:
            res=5
        elif dif1==-1 and dif2==0:
            res=6
        elif dif1==-1 and dif2==1:
            res=7
        return res
