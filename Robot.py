#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

#import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        dedo = ??
        self.R = 1.5*dedo
        #self.L = ??
        #self. ...
        self.eje_rueda = 1 # TODO: CAMBIAR

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        self.BP.offset_motor_encoder(self.BP.PORT_A,
            self.BP.get_motor_encoder(self.BP.PORT_A))
        self.BP.offset_motor_encoder(self.BP.PORT_D,
            self.BP.get_motor_encoder(self.BP.PORT_D))

        ##################################################
        # odometry shared memory values
        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.th = Value('d',0.0)
        self.finished = Value('b',1) # boolean to show if odometry updates are finished

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        #self.lock_odometry.acquire()
        #print('hello world', i)
        #self.lock_odometry.release()

        # odometry update period --> UPDATE value!
        self.P = 1.0



    def setSpeed(self, v,w):
        """ To be filled - These is all dummy sample code """
        print("setting speed to %.2f %.2f" % (v, w))
        if w==0:
            vI = vD = v
        else if v == 0:
            vI = w*self.eje_rueda
            vD = -vI
        else:
            rGiro = v/w
            vI = v * (1-self.eje_rueda/rGiro)
            vD = v * (1+self.eje_rueda/rGiro)
        wI = vI
        # compute the speed that should be set in each motor ...

        #speedPower = 100
        #BP.set_motor_power(BP.PORT_B + BP.PORT_C, speedPower)

        speedDPS_left = 180
        speedDPS_right = 180
        #self.BP.set_motor_dps(self.BP.PORT_B, speedDPS_left)
        #self.BP.set_motor_dps(self.BP.PORT_C, speedDPS_right)


    def readSpeedIzqDcha(self):
        """ En DPS (Degrees P S)"""
        izq = self.BP.get_motor_dps(self.BP.PORT_B)
        dcha = self.BP.get_motor_dps(self.BP.PORT_C)
        return izq, dcha

    def readSpeed(self):
        """ En DPS (Degrees P S)"""
        pass

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=()) #additional_params?))
        self.p.start()
        print("PID: ", self.p.pid)

    def deltaX(self, deltaSi, deltaTh):
        # readSpeed debe devolver v,w
        return deltaSi*np.cos(self.th.value + deltaTh/2.0)

    def deltaY(self, deltaSi, deltaTh):
        return deltaSi*np.sin(self.th.value + deltaTh/2.0)

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self): #, additional_params?):
        """ To be filled ...  """

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # compute updates
            deltaTh += deltaTH()
            v, w = self.readSpeed()
            deltaSi = v/w*deltaTh
            self.x.value += deltaX(deltaSi,deltaTh)
            self.y.value += deltaY(deltaSi,deltaTh)
            self.th.value += deltaTh
            ######## UPDATE FROM HERE with your code (following the suggested scheme) ########
            sys.stdout.write("Dummy update of odometry ...., X=  %d, \
                Y=  %d, th=  %d \n" %(self.x.value, self.y.value, self.th.value) )
            #print("Dummy update of odometry ...., X=  %.2f" %(self.x.value) )

            # update odometry uses values that require mutex
            # (they are declared as value, so lock is implicitly done for atomic operations, BUT =+ is NOT atomic)

            # Operations like += which involve a read and write are not atomic.
            with self.x.get_lock():
                self.x.value+=1

            # to "lock" a whole set of operations, we can use a "mutex"
            self.lock_odometry.acquire()
            #self.x.value+=1
            self.y.value+=1
            self.th.value+=1
            self.lock_odometry.release()

            try:
                # Each of the following BP.get_motor_encoder functions returns the encoder value
                # (what we want to store).
                sys.stdout.write("Reading encoder values .... \n")
                #[encoder1, encoder2] = [self.BP.get_motor_encoder(self.BP.PORT_B),
                #    self.BP.get_motor_encoder(self.BP.PORT_C)]
            except IOError as error:
                #print(error)
                sys.stdout.write(error)

            #sys.stdout.write("Encoder (%s) increased (in degrees) B: %6d  C: %6d " %
            #        (type(encoder1), encoder1, encoder2))


            # save LOG
            # Need to decide when to store a log with the updated odometry ...

            ######## UPDATE UNTIL HERE with your code ########


            tEnd = time.clock()
            time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))


    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        #self.BP.reset_all()
