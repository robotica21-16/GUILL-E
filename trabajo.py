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

baldosa=0.4
def mapA(robot):
    t = TrayectoriaTrabajoA(baldosa-0.02)
    robot.setTrajectory(t)
    robot.executeTrajectory()

def mapB(robot):
    t = TrayectoriaTrabajoB(baldosa-0.02)
    robot.setTrajectory(t)
    robot.executeTrajectory()


def main(args):
    """    
    """
    try:
        robot = Robot()
        if args.test_r2d2:
            if robot.detectR2D2():
                print("veo a r2d2")
            else:
                print("No veo a r2d2")

        if args.test_suelo:
            robot.startOdometry()
            while True:
                if robot.colorSensorBlack():
                    print("Es negro")
                    
        #robot.executePath_neigh()
        robot.stopOdometry()


        if args.trabajo:
            robot = Robot()
            robot.startOdometry()
            if robot.colorSensorBlack():
                mapB(robot)
            else:
                mapA(robot)


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
    parser.add_argument("-t", "--trabajo", help="execute all map",
                        default="False")
    parser.add_argument("-g", "--test_go", help="test go function",
                        default="")
    parser.add_argument("-u", "--test_ultrasound", help="test ultrasound sensor",
                        default=False)


    parser.add_argument('-td','--test_r2d2', help="test image recognition", dest='test_r2d2', action='store_true')
    parser.add_argument('-ts','--test_suelo', help="test suelo negro recognition", dest='test_suelo', action='store_true')

    # parser.add_argument('-npt', '--no-plottrajectory', dest='plot_trajectory', action='store_false')
    parser.set_defaults(test_suelo=False)

    args = parser.parse_args()
    main(args)
