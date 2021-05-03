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


def mapA(robot):
    #t = TrayectoriaTrabajoA(baldosa-0.02)
    mapa = Map2D("trabajo/mapaA_CARRERA.txt")
    return mapa

if __name__ == "__main__":
	
	try:
		robot = Robot()
		robot.useGyro = False
		robot.setMapNoPath(mapA(robot))
		x = [4.5, 4]
		baldosa = robot.mapa.sizeCell / 1000.0
		print(baldosa)
		robot.setOdometry([x[0] * baldosa,x[1]*baldosa, math.pi])
		
		robot.startOdometry()
		
		
		#
		distObj = baldosa/2
		robot.relocateWithSonar(math.pi, [(3.5*baldosa), None, math.pi], distance2=distObj*100.0, eps=0.1)
		#
		
		#robot.relocateWithSonar(math.pi, [15, None, math.pi])
		
		odo = robot.readOdometry()
		robot.go(odo[0], odo[1]+baldosa*3.0)
		robot.stopOdometry()
	except KeyboardInterrupt:
		# except the program gets interrupted by Ctrl+C on the keyboard.
		# THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
		robot.stopOdometry()
	except BaseException as e:
		print(e)
		traceback.print_exc()
		robot.stopOdometry()

