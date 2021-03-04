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
        if args.trayectoria == 1:
                d = 0.2
                t1 = Trayectoria1(d)
                robot.setTrajectory(t1)
                robot.executeTrajectory()
        elif args.trayectoria == 3:
                d = 1
                t3 = Trayectoria3(d)
                robot.setTrajectory(t3)
                robot.executeTrajectory()
                
        else:
                r1 = 0.2
                r2 = 0.25
                d = 1
                t2 = Trayectoria2(r1, r2, d)
                robot.setTrajectory(t2)
                robot.executeTrajectory()
        


        # DUMMY CODE! delete when you have your own
        robot.setSpeed(0,0)
        print("Start : %s" % time.ctime())
        time.sleep(3)
        print("X value from main tmp %d" % robot.x.value)
        time.sleep(3)
        print("End : %s" % time.ctime())

        robot.lock_odometry.acquire()
        print("Odom values at main at the END: %.2f, %.2f, %.2f " % (robot.x.value, robot.y.value, robot.th.value))
        robot.lock_odometry.release()

        # PART 1:
        # robot.setSpeed()
        # until ...

        # PART 2:
        # robot.setSpeed()
        # until ...

        # ...



        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
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
    parser.add_argument("-t", "--trayectoria", help="Elige la trayectoria a dibujar (1 o 2)",
                                            type=int, default=1)
    
    args = parser.parse_args()

    main(args)



