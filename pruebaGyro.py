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

def mapA(robot):
    t = TrayectoriaTrabajoA(baldosa-0.02)
    mapa = Map2D("trabajo/mapaA_CARRERA.txt")
    return t, mapa

def mapB(robot):

    t = TrayectoriaTrabajoB(baldosa)
    mapa = Map2D("trabajo/mapaA_CARRERA.txt")
    return t, mapa

def main(args):
    """
    """
    robot = Robot()
    try:
        while(True):
            print(robot.angleGyro())
            
            time.sleep(1)


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

    args = parser.parse_args()
    main(args)
