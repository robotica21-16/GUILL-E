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


def main(args):
    """
    Example to load "mapa1.txt"
    """

    try:
        if args.test_go!="":
            robot = Robot()
            robot.startOdometry()
            robot.go(0,0.4)
            robot.go(0.4,0.4)
            robot.go(0.4,0)
            robot.go(0,0)
            robot.stopOdometry()
            exit(0)
        if args.test_ultrasound:
            robot = Robot()
            robot.testDistance()
            exit(0)

        if not os.path.isfile(args.mapfile):
            print('Map file %s does not exist' % args.mapfile)
            exit(1)

        map_file = args.mapfile;
        

        # 1. load map and compute costs and path
        myMap = Map2D(map_file)
        
        matplotlib.pyplot.close('all')


        x1,y1,th1 = 0,0,math.pi/2.0
        x2,y2 = 2,0

        robot = Robot()
        robot.setMap(myMap, [x1,y1,th1], [x2,y2])
        robot.startOdometry()
        robot.executePath()
        #robot.executePath_neigh()
        robot.stopOdometry()
        myMap.drawMap(saveSnapshot=False)

    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()
    except BaseException as e:
        print(e)
        traceback.print_exc()
        robot.stopOdometry()



if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mapfile", help="path to find map file",
                        default="mapa1.txt")
    parser.add_argument("-g", "--test_go", help="test go function",
                        default="")
    parser.add_argument("-u", "--test_ultrasound", help="test ultrasound sensor",
                        default=False)
    args = parser.parse_args()
    main(args)
