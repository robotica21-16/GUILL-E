#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import cv2
import numpy as np
import time
from Robot import Robot
from camera.color_blobs import *

def main(args):
    try:

        #print("Antes")
        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot() 
        # 1. launch updateOdometry thread()
        robot.startOdometry()

        # 2. Loop running the tracking until it is near enough, then catch the ball
        
        if args.search or args.file:
            search_blobs(robot.cam, args.file, show=True)
        elif args.picture:
            robot.takePicture()
        elif args.debug_continuous:
            robot.detect_continuous()
        elif args.track:
            robot.trackBall()
        elif args.catch:
            robot.closing = True
            robot.moveClaws()
            #robot.moveClaws()

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
    parser.add_argument("-c", "--catch", help="catch the ball",
                        type=bool, default=False)
    parser.add_argument("-s", "--search", help="Search blobs",
                    type=bool, default=False)
    
    parser.add_argument("-p", "--picture", help="Take picture",
                    type=bool, default=False)
    parser.add_argument("-f", "--file", help="Detect blobs from file",
                    type=str, default=None)
    parser.add_argument("-d", "--debug_continuous", help="Continuous",
                    type=bool, default=False)
    parser.add_argument("-t", "--track", help="Track ball",
                    type=bool, default=False)
    args = parser.parse_args()

    main(args)


