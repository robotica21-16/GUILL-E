#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import cv2
import numpy as np
import time
from Robot import Robot
from p3.color_blobs import *

def main(args):
    try:

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot() 
        # 1. launch updateOdometry thread()
        #robot.startOdometry()

        # 2. Loop running the tracking until ??, then catch the ball
        # TO-DO: ADD to the Robot class a method to track an object, given certain parameters
        # for example the different target properties we want (size, position, color, ..)
        # or a boolean to indicate if we want the robot to catch the object or not
        # At least COLOR, the rest are up to you, but always put a default value.
    	# res = robot.trackObject(colorRangeMin=[0,0,0], colorRangeMax=[255,255,255], 
        #                   targetSize=??, target??=??, ...)
        #robot.trackObject(args.view)
        if args.search or args.file:
            search_blobs(robot.cam, args.file)
        elif args.picture:
            robot.takePicture()
        elif args.debug_continuous:
            robot.detect_continuous()

        # if res:
        #   robot.catch

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
    parser.add_argument("-c", "--color", help="color of the ball to track",
                        type=float, default=40.0)
    parser.add_argument("-v", "--view", help="Ver camera",
                    type=bool, default=False)
    parser.add_argument("-s", "--search", help="Search blobs",
                    type=bool, default=False)
    
    parser.add_argument("-p", "--picture", help="Take picture",
                    type=bool, default=False)
    parser.add_argument("-f", "--file", help="Detect blobs from file",
                    type=str, default=None)
    parser.add_argument("-d", "--debug_continuous", help="Continuous",
                    type=bool, default=False)
    args = parser.parse_args()

    main(args)


