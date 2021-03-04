#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys

from geometry.geometry import *

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

        self.R_rueda = 4
        self.eje_rueda = 1 # TODO: CAMBIAR
        self.L = 2*self.eje_rueda
        #self. ...

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

        self.f_log = open("logs/log.txt","a")#append
        fila = ["t", "x", "y", "th"]
        self.f_log.write("\t".join([str(e) for e in fila]) + "\n")


    def setSpeed_old(self, v, w):
        """ To be filled - These is all dummy sample code """
        #print("setting speed to %.2f %.2f" % (v, w))
        if w==0:
            vI = vD = v
        elif v == 0:
            vI = w*self.eje_rueda
            vD = -vI
        else:
            rGiro = v/w
            vI = v * (1-self.eje_rueda/rGiro)
            vD = v * (1+self.eje_rueda/rGiro)
        # Velocidad lineal/long de la rueda, en rad:
        wI = vI/(self.R_rueda*2.0*math.pi)
        wD = vD/(self.R_rueda*2.0*math.pi)

        #speedPower = 100
        #BP.set_motor_power(BP.PORT_B + BP.PORT_C, speedPower)

        speedDPS_left = wI/math.pi*180
        speedDPS_right = wD/math.pi*180
        self.BP.set_motor_dps(self.BP.PORT_A, speedDPS_left)
        self.BP.set_motor_dps(self.BP.PORT_D, speedDPS_right)


    def setSpeed(self, v, w):
        """ To be filled - These is all dummy sample code """
        # wID = [wI, wD]:
        wID = izqDchaFromVW(self.R_rueda, self.L, v, w)
        wI = wID[0]
        wD = wID[1]
        speedDPS_left = wI/math.pi*180
        speedDPS_right = wD/math.pi*180
        self.BP.set_motor_dps(self.BP.PORT_A, speedDPS_left)
        self.BP.set_motor_dps(self.BP.PORT_D, speedDPS_right)


    def readSpeedIzqDcha(self, dT):
        """ En DPS (Degrees P S)"""
        #izq = self.BP.get_motor_encoder(self.BP.PORT_B)
        #dcha = self.BP.get_motor_encoder(self.BP.PORT_C)
        #return izq, dcha
        try:
            # Each of the following BP.get_motor_encoder functions returns the encoder value
            # (what we want to store).
            #sys.stdout.write("Reading encoder values .... \n")
            izq = self.BP.get_motor_encoder(self.BP.PORT_A)
            dcha = self.BP.get_motor_encoder(self.BP.PORT_D)
            return izq/dT, dcha/dT
        except IOError as error:
            #print(error)
            sys.stdout.write(error)

    def readSpeed(self, dT):
        """ En DPS (Degrees P S)"""
        izq, dcha = self.readSpeedIzqDcha(dT)
        wI = norm_pi_deg(izq)
        wD = norm_pi_deg(dcha)
        v,w = vWiFromIzqDcha(self.R_rueda, self.L, wI, wD)
        return v,w

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=()) #additional_params?))
        self.p.start()
        print("PID: ", self.p.pid)

    def deltaTH(self):
        pass

    def deltaX(self, deltaSi, deltaTh):
        # readSpeed debe devolver v,w
        return deltaSi*np.cos(self.th.value + deltaTh/2.0)

    def deltaY(self, deltaSi, deltaTh):
        return deltaSi*np.sin(self.th.value + deltaTh/2.0)

    def writeLog(self, sep="\t"):
        fila = [self.tLast, self.x.value, self.y.value, self.th.value]
        print("hola", fila)
        self.f_log.write("\t".join([str(e) for e in fila])+"\n")
        self.f_log.flush()

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self): #, additional_params?):
        """ To be filled ...  """
        self.tLast = time.clock()
        time.sleep(0.1)
        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()
            dT = tIni-self.tLast
            self.tLast = tIni

            # compute updates
            # deltaTh += self.deltaTH()
            v, w = self.readSpeed(dT)
            if w != 0:
                deltaSi = v/w*deltaTh
            else:
                deltaSi = v*dT
            deltaTh = norm_pi(w*dT)
            self.lock_odometry.acquire()

            self.x.value += self.deltaX(deltaSi,deltaTh)
            self.y.value += self.deltaY(deltaSi,deltaTh)
            self.th.value = norm_pi(self.th.value+deltaTh)
            self.lock_odometry.release()

            # ######## UPDATE FROM HERE with your code (following the suggested scheme) ########
            # sys.stdout.write("Dummy update of odometry ...., X=  %d, \
            #     Y=  %d, th=  %d \n" %(self.x.value, self.y.value, self.th.value) )
            # #print("Dummy update of odometry ...., X=  %.2f" %(self.x.value) )
            #
            # # update odometry uses values that require mutex
            # # (they are declared as value, so lock is implicitly done for atomic operations, BUT =+ is NOT atomic)
            #
            # # Operations like += which involve a read and write are not atomic.
            # with self.x.get_lock():
            #     self.x.value+=1
            #
            # # to "lock" a whole set of operations, we can use a "mutex"
            # self.lock_odometry.acquire()
            # #self.x.value+=1
            # self.y.value+=1
            # self.th.value+=1
            # self.lock_odometry.release()



            #sys.stdout.write("Encoder (%s) increased (in degrees) B: %6d  C: %6d " %
            #        (type(encoder1), encoder1, encoder2))


            # save LOG
            # Need to decide when to store a log with the updated odometry ...
            self.writeLog()
            ######## UPDATE UNTIL HERE with your code ########


            tEnd = time.clock()
            time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))


    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        self.f_log.close()
        #self.BP.reset_all()
