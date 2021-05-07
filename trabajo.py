#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import numpy as np
import time
import traceback

import matplotlib
matplotlib.use("TkAgg")
# sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

from Robot import Robot
from p4.MapLib import Map2D
import math

from trayectorias.trayectorias import *

baldosa=0.4
baldosa_nestor = 0.2

def mapA(robot):
    #t = TrayectoriaTrabajoA(baldosa-0.02)
    t = TrayectoriaTrabajoARelativa(baldosa)
    mapa = Map2D("trabajo/mapaA_CARRERA.txt")
    return t, mapa

def mapB(robot):

    t = TrayectoriaTrabajoBRelativa(baldosa)
    mapa = Map2D("trabajo/mapaB_CARRERA.txt")
    return t, mapa


def relocateRobot(robot, fin, executingMapA, baldosa):
    if fin[0] == 3: #pared izq:
        thObj = math.pi
        xObj = (3.5*baldosa)
        if not executingMapA:
            thObj = 0
    else:#dcha
        print("fin0 es ", fin[0])
        thObj = 0
        xObj = (6.5*baldosa)
        if not executingMapA:
            thObj = math.pi
            xObj = 0.5*baldosa

    # sonar
    print("going to", xObj, "th:",thObj)
    distObj = baldosa/2
    robot.relocateWithSonar(thObj, [xObj, None, thObj], distance2=distObj*100.0, eps=0.1)


def main(args):
    """
    Presentation main program, and some debugging options
    """
    try:
        robot = Robot()
        if args.giros != -1:
            robot.useGyro=True
            t = TrayectoriaGiros(args.giros, args.rad*math.pi)
            #t = TrayectoriaGirosAbs(args.giros, args.rad*math.pi)
            robot.startOdometry()
            robot.setTrajectory(t)
            robot.executeTrajectory()
            robot.stopOdometry()
            exit(0)

        if args.test_r2d2:
            while True:
                if robot.detectR2D2(verbose=True, DEBUG=1):
                    print("veo a r2d2")
                else:
                    print("No veo a r2d2")
                time.sleep(3)

        if args.test_suelo:
            while True:
                print(robot.colorSensorValue())
                if robot.colorSensorBlack():
                    print("Es negro")
                elif robot.colorSensorWhite():
                    print("Es blanco")
                else:
                    print("Ni negro ni blanco")
                time.sleep(1)

        elif args.trabajo or args.test_map or args.pelota or args.test_r2_section:
            # Choose map (A or B) based on color sensor:
            if not robot.colorSensorBlack():
                print("Estoy en el mapa A")
                t, mapa = mapA(robot)
                celdaIni = [1,2,-math.pi/2]
                #fin = [3,3]
                fin = [4,6]
                ini=[1,6, -math.pi/2]
                executingMapA = True
            else:
                print("Estoy en el mapa B")
                t, mapa = mapB(robot)
                celdaIni = [5,2,-math.pi/2]
                fin = [2,6]
                ini=[5,6, -math.pi/2]
                executingMapA = False

            if args.trabajo: # full demo
                robot.setMapNoPath(mapa) # set map
                x, y = robot.posFromCell(ini[0], ini[1])
                robot.setOdometry([x, y, ini[2]]) # set initial pos
                robot.startOdometry()
                robot.setTrajectory(t) # set S trajectory
                robot.executeTrajectory() # execute it
                robot.useGyro = False # stop using Gyro (has more error than odo later)
                robot.setPath(celdaIni, fin) # calculate the path in the map
                nBaldosas = 2.9 if executingMapA else 3.9 # x value of vertical line
                # wait for the two lines and update odo accordingly:
                robot.waitForWhite([0,1], [nBaldosas * baldosa, 3 * baldosa])
                # execute the planned path:
                robot.executePath()
            ### debugging options ###
            elif args.test_map or args.test_r2_section: # only map
                if args.test_r2_section:
                    celdaIni = [3, 2, math.pi/2]
                nBaldosas = 3 if executingMapA else 4
                robot.waitForWhite([0,1], [nBaldosas * baldosa, 3 * baldosa])
                robot.setMap(mapa,celdaIni, fin)
                #robot.mapa.drawMap(saveSnapshot=False)
                robot.startOdometry()
                robot.executePath()
            if args.pelota or args.sonar:
                robot.startOdometry()
            ### end debugging options ###
            if args.trabajo or args.test_r2_section or args.test_map:
                # relocate a bit further down to see the homography better:
                x,y = robot.posFromCell(fin[0], fin[1]-0.5)
                robot.go(x,y)
                # orient GUILLE to our side 45 degrees (map A:left, B:right)
                dX, dY = -0.05, 0.05
                wHomography = -0.1 # rotate right if a, left if b
                if not executingMapA: # map b to the other side
                    dX = -dX
                    wHomography = -wHomography
                odo = robot.readOdometry()
                robot.go(odo[0]+dX, odo[1]+dY) # face the pictures
                res = 0 # result of homography
                while res == 0:
                    res = robot.detectHomography() # take picture, compare, etc.
                    robot.setSpeed(0, wHomography) # turn
                    time.sleep(0.01) # wait a bit
                # res = 0 -> no detection
                # res = 1 -> r2
                # res = 2 -> the other one
                if res == 1: # r2
                    print("detected r2")
                    # choose the exit path according to the map being executed:
                    if executingMapA:
                        fin[0] -= 1
                    else:
                        fin[0]-=2
                else: # el otro
                    print("detected el otro")
                    # choose the exit path according to the map being executed:
                    if executingMapA:
                        fin[0] += 2
                    else:
                        fin[0]+=1
                print("going to", fin)
            #track and catch ball:
            if not args.sonar:
                robot.trackBall()

            #sonar, update th and y odometry values based on the wall:
            relocateRobot(robot, fin, executingMapA, baldosa)

            # go to last cell:
            robot.setPathFromCurrentPosition(fin)
            robot.executePath(checkObstacles=False)
            # exit the map:
            x,y = robot.posFromCell(fin[0], fin[1]+1)
            robot.go(x,y, checkObstacles=False)

            robot.stopOdometry()

    except KeyboardInterrupt:
        robot.stopOdometry()
    except BaseException as e:
        print(e)
        traceback.print_exc()
        robot.stopOdometry()

if __name__ == "__main__":
    # get and parse arguments passed to main
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--trabajo", help="execute all map", dest='trabajo', action='store_true')

    parser.set_defaults(trabajo=False)
    parser.add_argument("-g", "--test_go", help="test go function",
                        default="")
    parser.add_argument("-u", "--test_ultrasound", help="test ultrasound sensor",
                        default=False)


    parser.add_argument('-td','--test_r2d2', help="test image recognition", dest='test_r2d2', action='store_true')
    parser.add_argument('-ts','--test_suelo', help="test suelo negro recognition", dest='test_suelo', action='store_true')
    parser.add_argument('-m', '--test_map', help='test only the map portion', dest='test_map', action='store_true')
    parser.add_argument('-r2', '--test_r2_section', help='test only the r2 portion', dest='test_r2_section', action='store_true')

    parser.add_argument('-p', '--pelota', help='test only the pelota', dest='pelota', action='store_true')
    parser.add_argument('-tg', '--giros', help='test giros',
                                            type=int, default=-1)
    parser.add_argument('-s', '--sonar', help='test sonar', dest='sonar',action='store_true')
    parser.add_argument('-rad', '--rad', help='radians ', type=float, default=2)
    parser.set_defaults(test_suelo=False)
    parser.set_defaults(test_r2d2=False)
    parser.set_defaults(sonar=False)

    parser.set_defaults(test_r2_section=False)
    parser.set_defaults(test_map=False)
    parser.set_defaults(pelota=False)

    args = parser.parse_args()
    main(args)
