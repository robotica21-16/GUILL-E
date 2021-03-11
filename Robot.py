#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys

import picamera
from picamera.array import PiRGBArray

import datetime
import numpy as np

from geometry.geometry import *

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

from p3.color_blobs import search_blobs




class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

        # Robot construction parameters

        self.R_rueda = 0.027
        self.L = 0.140
        self.eje_rueda = self.L/2.0

        # Camera Initialization
        self.cam = picamera.PiCamera()
        self.cam.resolution = (320, 240)
        #self.cam.resolution = (640, 480)
        self.cam.framerate = 32
        rawCapture = PiRGBArray(self.cam, size=(320, 240))
        #rawCapture = PiRGBArray(self.cam, size=(640, 480))

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        self.wmax = math.pi/3
        self.vmax = 1.0/8.0
        self.vTarget = self.vmax/2
        self.wTarget = self.wmax/2


        self.targetArea = 1 # TODO: poner bien!!!!!

        # Configure sensors, for example a touch sensor.
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        self.ruedaIzq = self.BP.PORT_D
        self.ruedaDcha = self.BP.PORT_A
        self.motorGarras = self.BP.PORT_C
        self.maxRotGarras = 30 # angulo de giro de las garras 30 grados, comprobar!
        self.BP.offset_motor_encoder(self.ruedaIzq,
            self.BP.get_motor_encoder(self.ruedaIzq))
        self.BP.offset_motor_encoder(self.ruedaDcha,
            self.BP.get_motor_encoder(self.ruedaDcha))
        self.BP.offset_motor_encoder(self.motorGarras,
            self.BP.get_motor_encoder(self.motorGarras))

        self.rotIzqDeg = 0
        self.rotDchaDeg = 0

        ##################################################
        # odometry shared memory values
        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.th = Value('d',0.0)
        self.finished = Value('b',1) # boolean to show if odometry updates are finished


        self.tLast = time.perf_counter()
        self.tInitialization = self.tLast

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        # odometry update period --> UPDATE value!
        self.P = 0.05

        self.f_log = open("logs/log.txt","a")#append
        fila = ["t", "x", "y", "th"]
        self.f_log.write(str(datetime.datetime.now()))
        self.f_log.write("\t".join([str(e) for e in fila]) + "\n")

    def setSpeed(self, v, w):
        """
        Sets the speed as degrees PS to each motor according to
        v (linear) and w (angular)
        """
        # wID = [wI, wD]:
        if v == 0 and False:
            vI = w*self.eje_rueda
            vD = -vI
            wI = vI*(self.R_rueda*2.0*math.pi)
            wD = vD*(self.R_rueda*2.0*math.pi)
        else:
            wID = izqDchaFromVW(self.R_rueda, self.L, v, w)
            wI = wID[0]
            wD = wID[1]
        speedDPS_left = wI/math.pi*180
        speedDPS_right = 0.9995*wD/math.pi*180
        self.BP.set_motor_dps(self.ruedaIzq, speedDPS_left)
        self.BP.set_motor_dps(self.ruedaDcha, speedDPS_right)

    def setTrajectory(self,trajectory):
        """
        Sets the trajectory to perform (class geometry.Trajectory)
        """
        self.trajectory = trajectory

    def executeTrajectory_time(self):
        """
        Executes the saved trajectory (sequence of v,w and t)
        """
        for move in self.trajectory.movements:
            self.setSpeed(move.vc[0], move.vc[1])
            time.sleep(move.t)

    def closeEnough(self, target, eps=np.array([0.01, 0.01, 0.2])):
        odo = self.readOdometry()
        close = False
        if target[0] == None and target[1] == None:
            print(target[2], " ---- ", odo[2])
            if abs((((target[2])) - (norm_pi(odo[2])))) < eps[2]:
                close = True
        elif target[2] != None:
                
            if abs(norm_pi(target[0] - odo[0])) < eps and abs(norm_pi(target[1] - odo[1])) < eps:
                close = True
        return close

    def executeTrajectory(self):
        """
        Executes the saved trajectory (sequence of v,w and t)
        """
        period = 0.005
        for i in range(0, len(self.trajectory.targetPositions)):
            # target = [x,y,th]
            # pos = [x,y,th]
            v = self.trajectory.targetV[i]
            w = self.trajectory.targetW[i]
            self.setSpeed(v, w)
            while not self.closeEnough(self.trajectory.targetPositions[i]):
                tIni = time.perf_counter()
                tFin = time.perf_counter()
                time.sleep(period-(tFin-tIni))
                #tIni = time.perf_counter()
                #v,w = geometry.fromPosToTarget(np.array(self.readOdometry()),
                #        target, self.vTarget, self.wTarget)
                #self.setSpeed(v, w)
                #tFin = time.perf_counter()
                #time.sleep(period-(tFin-tIni))
        self.setSpeed(0,0)


    def readSpeedIzqDcha(self, dT):
        """
        Returns vL, vD as DPS of the wheels (left, right)
        """
        #izq = self.BP.get_motor_encoder(self.BP.PORT_B)
        #dcha = self.BP.get_motor_encoder(self.BP.PORT_C)
        #return izq, dcha
        try:
            # Each of the following BP.get_motor_encoder functions returns the encoder value
            # (what we want to store).
            #sys.stdout.write("Reading encoder values .... \n")
            izq = self.BP.get_motor_encoder(self.ruedaIzq)
            dcha = self.BP.get_motor_encoder(self.ruedaDcha)
            izq_bk = izq
            dcha_bk = dcha
            izq = izq-self.rotIzqDeg
            dcha = dcha-self.rotDchaDeg
            #print("ideg, ddeg: ", self.rotIzqDeg, self.rotDchaDeg)

            self.rotIzqDeg = izq_bk
            self.rotDchaDeg = dcha_bk
            #print("i, d (sin dT): ", izq, dcha, dT)
            return izq/dT, dcha/dT
        except IOError as error:
            #print(error)
            sys.stdout.write(error)

    def readSpeed(self, dT):
        """
        In: dT = delta of time
        Returns v,w (in m/s and rad/s) according to the motor encoders
        """
        #print('-----------------------------------')
        izq, dcha = self.readSpeedIzqDcha(dT)
        #print("i, d: ", izq, dcha)
        wI = np.radians(izq)
        wD = np.radians(dcha)
        print("wi, wd: ", wI, wD)
        v,w = vWiFromIzqDcha(self.R_rueda, self.L, wI, wD)
        print("v, w: ", v, w)
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


    def deltaX(self, deltaSi, deltaTh):
        """
        In:
            deltaSi: variation in space (tangent) since last timestamp
            deltaTh: variation in angle since last
        Returns:
            deltaX: variation in X coordinate (local)
        """
        # readSpeed debe devolver v,w
        return deltaSi*np.cos(self.th.value + deltaTh/2.0)

    def deltaY(self, deltaSi, deltaTh):
        """
        In:
            deltaSi: variation in space (tangent) since last timestamp
            deltaTh: variation in angle since last
        Returns:
            deltaY: variation in Y coordinate (local)
        """
        return deltaSi*np.sin(self.th.value + deltaTh/2.0)

    def writeLog(self, v,w, deltaTh, deltaSi,sep="\t"):
        """
        Writes a row of the log (t, x, y, th, v, w, deltaTh, deltaSi)
        (each with 2 decimal positions, v multiplied by 100 to gain precision)
        """
        fila = [self.tLast-self.tInitialization, self.x.value,
        self.y.value, self.th.value, 100*v,w, deltaTh, deltaSi]
        fila = "\t".join(['{0:.2f}'.format(e) for e in fila])+"\n"
        #print(fila)
        self.f_log.write(fila)
        self.f_log.flush()

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self): #, additional_params?):
        """
        Updates self.x, .y and .th according to the change in the motor encoders
        Writes a row in the log
        """
        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.perf_counter()
            dT = tIni-self.tLast
            self.tLast = tIni

            # compute updates
            # deltaTh += self.deltaTH()
            v, w = self.readSpeed(dT)
            deltaTh = norm_pi(w*dT)
            eps = 0.01
            print("----------\n", deltaTh)
            if -eps < deltaTh < eps:
                print("a")
                deltaSi = v*dT
            else:
                print("b")
                deltaSi = v/w*deltaTh
            print("deltaSi: ", deltaSi)
            self.lock_odometry.acquire()

            self.x.value += self.deltaX(deltaSi,deltaTh)
            self.y.value += self.deltaY(deltaSi,deltaTh)
            self.th.value = norm_pi(self.th.value+deltaTh)
            self.lock_odometry.release()

            self.writeLog(v,w, deltaTh, deltaSi)

            tEnd = time.perf_counter()
            time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))

    def horizontalDistance(kp, obj=[0,0]):
        return math.abs(kp[0], obj[0])

    def targetW(self, d, dmin=-320/2, dmax=320/2):
        return np.interp(d, [-dmin, dmax], [-self.wmax, +self.wmax])

    def targetV(self, A):
        """
        ...
        """
        # cuando A=0 (en el infinito) -> targetArea-A = targetArea -> v=vmax
        # cuando A=a (en el objetivo) -> targetArea-A = 0 -> v = 0
        return np.interp(self.targetArea-A, [self.targetArea, 0], [self.vmax, 0])

    def trackObject(self, colorRangeMin=[0,0,0], colorRangeMax=[255,255,255]):
        # targetSize=??, target??=??, catch=??, ...)
        # targetFound = False
        targetPositionReached = False
        finished = False
        while not finished:
            # 1. search the most promising blob ..
            kp = search_blobs(self.cam)
            while not targetPositionReached:
                # 2. decide v and w for the robot to get closer to target position

                d = horizontalDistance(kp, [0,0])
                r = kp.size/2 # suponiendo que es el diametro
                A = r**2 * math.pi
                w = self.targetW(d)
                v = self.targetV(A)
                eps = 0.01
                if self.targetArea-eps > A > self.targetArea+eps:
                    targetPositionReached  = True
                    finished = True
                    return finished

    def catch(self):
        self.catcher = Process(target=self.catchRoutine, args=()) #additional_params?))
        self.catcher.start()
        # decide the strategy to catch the ball once you have reached the target position
        pass

    def catchRoutine(self):
        """
        Rutina paralela para cerrar las garras
        """
        DPS = 20 # 20º por segundo
        end = False
        period = 0.2
        while not self.finished.value and not end:
            tIni = time.perf_counter()
            end = -self.maxRotGarras < self.BP.get_motor_encoder(self.motorGarras) < self.maxRotGarras
            if not end:
                self.BP.set_motor_dps(self.motorGarras, DPS)
                tEnd = time.perf_counter()
                time.sleep(period - (tEnd-tIni))


    # Stop the odometry thread.
    def stopOdometry(self):
        """
        Stops odometry and sets the speed to 0
        """
        self.finished.value = True
        self.f_log.close()
        self.setSpeed(0,0)
        self.BP.set_motor_dps(self.motorGarras, 0)
        #self.BP.reset_all()
