#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot

from p2.trayectorias import *


def main(args):
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)

        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()

        print("X value at the beginning from main X= %.2f" %(robot.x.value))

        # 1. launch updateOdometry Process()
        robot.startOdometry()

        # 2. perform trajectory
        tray = 3
        if args.control == "tiempo":
            if args.trayectoria == 1: # ocho
                    d = 0.2
                    t1 = Trayectoria1(d)
                    robot.setTrajectory(t1)
                    robot.executeTrajectory_time()
            elif args.trayectoria == 3: # linea recta
                    d = 1
                    t3 = Trayectoria3(d)
                    robot.setTrajectory(t3)
                    robot.executeTrajectory_time()

            else: # trayectoria 2, 2 radios
                    r1 = 0.2
                    r2 = 0.25
                    d = 1
                    t2 = Trayectoria2(r1, r2, d)
                    robot.setTrajectory(t2)
                    robot.executeTrajectory_time()
        else:
            if args.trayectoria == 1: # ocho
                    d = 0.2
                    t1 = Trayectoria1Posiciones(d)
                    robot.setTrajectory(t1)
                    robot.executeTrajectory()
            elif args.trayectoria == 3: # linea recta
                    d = 1
                    t3 = Trayectoria3Posiciones(d)
                    robot.setTrajectory(t3)
                    robot.executeTrajectory()

            else: # trayectoria 2, 2 radios
                    r1 = 0.2
                    r2 = 0.25
                    d = 1
                    t2 = Trayectoria2Posiciones(r1, r2, d)
                    robot.setTrajectory(t2)
                    robot.executeTrajectory()


        robot.setSpeed(0,0)
        robot.stopOdometry()


    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    parser.add_argument("-t", "--trayectoria", help="Elige la trayectoria a realizar (1, 2 o 3)",
                                            type=int, default=1)

    args = parser.parse_args()

    main(args)
