#!/usr/bin/env python
#
# https://www.dexterindustries.com/BrickPi/
# https://github.com/DexterInd/BrickPi3
#
# Copyright (c) 2016 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
# Modified by A. Murillo / L. Montano. 2018.
#
# This code is an example for running all motors while a touch sensor connected to PORT_1 of the BrickPi3 is being pressed.
# 
# Hardware: Connect EV3 or NXT motor(s) to two of the BrickPi3 motor ports. Make sure that the BrickPi3 is running on a 9v power supply.
#
# Results:  When you run this program, the motor(s) speed will ramp up AFTER the touch sensor is pressed. The position for each motor will be printed.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.TOUCH) # Configure for a touch sensor. If an EV3 touch sensor is connected, it will be configured for EV3 touch, otherwise it'll configured for NXT touch.

BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder B
BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) # reset encoder C

try:
    print("Press touch sensor on port 1 to run motors")
    value = 0
    while not value:
        try:
            value = BP.get_sensor(BP.PORT_1)
        except brickpi3.SensorError:
            pass
    
    speed = 0
    adder = 10
    finished = False
    running = False
    while not finished:
        # BP.get_sensor retrieves a sensor value.
        # BP.PORT_1 specifies that we are looking for the value of sensor port 1.
        # BP.get_sensor returns the sensor value.
        try:
            value = BP.get_sensor(BP.PORT_1)
        except brickpi3.SensorError as error:
            print(error)
            value = 0
        
        if value: # switch value of running every time we press the button
            running = not running
            
        if running:
            speed += adder
            if speed >= 100:
                finished = 1 # this is the last iteration
        else:   # else the touch sensor is not pressed or not configured, so set the speed to 0
            speed = 0

        # Set the motor speed for two motors
        BP.set_motor_power(BP.PORT_B + BP.PORT_C, speed)

        # optionally set a power limit (in percent) and a speed limit (in Degrees Per Second)
        #BP.set_motor_limits(BP.PORT_B, 80, 300)

        #Set the motor target speed in degrees per second
        #BP.set_motor_dps(BP.PORT_B + BP.PORT_C, 200)

        # set motor B's target position
        #BP.set_motor_position(BP.PORT_B, target)    

        # delay for 2 seconds, to keep current speed for a little bit
        time.sleep(2.00)  
        #BP.set_motor_power(BP.PORT_B + BP.PORT_C, 0)
        
        # Returns the encoder position in degrees

        try:
            # Each of the following BP.get_motor_encoder functions returns the encoder value 
            # (what we want to store).
            [encoder1, encoder2] = [BP.get_motor_encoder(BP.PORT_B), BP.get_motor_encoder(BP.PORT_C)]
        except IOError as error:
            print(error)

        try:
            BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder B
            BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) # reset encoder C
        except IOError as error:
            print(error)

        print("2 sec. at speed %3d --> Encoder increased (in degrees) B: %6d  C: %6d " % 
                (speed, encoder1, encoder2))
            
    
    # STOP the motors
    BP.set_motor_power(BP.PORT_B + BP.PORT_C, 0)

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
