#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''


import brickpi3 # import the BrickPi3 drivers
# except BaseException as e:
#     print(e)



import time     # import the time library for the sleep function
import sys

import cv2
import picamera
import datetime
import numpy as np
from picamera.array import PiRGBArray

from utils.utils import *

from geometry.geometry import *

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

from camera.color_blobs import *
from p4.MapLib import *


resolution=[320,240]

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """
        ##################################################
        # Robot construction parameters

        self.R_rueda = 0.027
        self.L = 0.140
        self.eje_rueda = self.L/2.0

        self.offset_right = 1.0#0.9995 # The way the bot is built, the left tire spins slightly slower than right

        ##################################################
        # Camera initialization
        self.cam = picamera.PiCamera()

        #self.cam.resolution = (320, 240)
        self.cam.resolution = tuple(resolution)
        self.cam.framerate = 60 # TODO: mirar maximo real
        #self.rawCapture = PiRGBArray(self.cam, size=(320, 240))
        self.rawCapture = PiRGBArray(self.cam, size=tuple(resolution))

        self.cam.rotation=180

        ##################################################
        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        ##################################################
        # Speed parameters
        self.wmax = math.pi/3
        self.vmax = 1.0/4.0
        self.vTarget = self.vmax
        self.wTarget = self.wmax  #/2

        self.rotIzqDeg = 0
        self.rotDchaDeg = 0

        ##################################################
        # Ball parameters
        self.ballArea = 110
        self.ballClawsArea = 60
        self.ballX = resolution[0]/2.0



        # Configure sensors, for example a touch sensor.
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        ##################################################
        # Sensors and motors
        # reset encoder B and C (or all the motors you are using)
        self.lock_garras = Lock()
        self.closing = Value('b',0)
        self.ruedaIzq = self.BP.PORT_D
        self.ruedaDcha = self.BP.PORT_A
        self.motorGarras = self.BP.PORT_C
        self.maxRotGarras = 70 # angulo de giro de las garras 30 grados
        self.BP.offset_motor_encoder(self.ruedaIzq,
            self.BP.get_motor_encoder(self.ruedaIzq))
        self.BP.offset_motor_encoder(self.ruedaDcha,
            self.BP.get_motor_encoder(self.ruedaDcha))
        #self.BP.offset_motor_encoder(self.motorGarras,
        #    self.BP.get_motor_encoder(self.motorGarras))
        ##Adjust Claws to be closed
        posClawsIni = self.BP.get_motor_encoder(self.motorGarras)
        if(posClawsIni < -5 or posClawsIni > 0):
            print("Adjusting claws")
            while(posClawsIni < -5):
                self.BP.set_motor_dps(self.motorGarras, 20)
                posClawsIni = self.BP.get_motor_encoder(self.motorGarras)
                #print(posClawsIni)
            while(posClawsIni > 0):
                self.BP.set_motor_dps(self.motorGarras, -20)
                posClawsIni = self.BP.get_motor_encoder(self.motorGarras)
                #print(posClawsIni)
        self.BP.set_motor_dps(self.motorGarras, 0)


        #################################################
        self.portSensorUltrasonic = self.BP.PORT_4
        self.BP.set_sensor_type(self.portSensorUltrasonic, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)
        self.min_celds=1  # cm


        ####################################################################################################
        # odometry shared memory values
        # usar parametro
        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.th = Value('d',0.0)
        self.finished = Value('b',1) # boolean to show if odometry updates are finished
        self.changeClaws = Value('b',1)

        self.tLast = time.perf_counter()
        self.tInitialization = self.tLast

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        # odometry update period
        self.P = 0.05

        ####################################################################################################
        # Logs
        now = datetime.datetime.now()
        self.f_log = open("logs/{:d}-{:d}-{:d}-{:02d}_{:02d}".format(now.year, now.month, now.day, now.hour, now.minute)+"-log.txt","a")#append
        fila = ["t", "x", "y", "th", "v", "w", "dTh", "dSi"]
        self.f_log.write("\t".join([str(e) for e in fila]) + "\n")
        time.sleep(0.2)

    ####################################################################################################
    # SPEED FUNCTIONS
    def setSpeed(self, v, w):
        """
        Sets the speed as degrees PS to each motor according to
        v (linear ms) and w (angular rad s)
        """
        # wID = [wI, wD]:

        wDI = izqDchaFromVW(self.R_rueda, self.L, v, w)
        wD = wDI[0]
        wI = wDI[1]

        speedDPS_left = wI/math.pi*180
        speedDPS_right = self.offset_right*wD/math.pi*180
        self.BP.set_motor_dps(self.ruedaIzq, speedDPS_left)
        self.BP.set_motor_dps(self.ruedaDcha, speedDPS_right)

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

        izq, dcha = self.readSpeedIzqDcha(dT)

        wI = np.radians(izq)
        wD = np.radians(dcha)

        v,w = vWiFromIzqDcha(self.R_rueda, self.L, wI, wD)
        #print("v, w: ", v, w)
        return v, w

    ####################################################################################################
    # Trajectories functions

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

    def closeEnough(self, target, w):
        odo = self.readOdometry()
        close = False
        if target[0] == None and target[1] == None and target[2] == None:
            close = True
        elif target[0] == None and target[1] == None:
            #print(target[2], " ---- ", odo[2])
            #if abs(norm_pi(target[2]-norm_pi(odo[2]))) < eps[2]:
            if reachedAngle(odo[2], target[2], w):
                return True
            #if w>0:
            #    close = odo[2] >= (target[2])
            #else:
            #    close = odo[2] <= (target[2])
        else:
            sinth = np.sin(odo[2])
            costh = np.cos(odo[2])
            cond1 = True
            cond2 = True
            if target[0] != None:
                cond1 = reached(odo[0],target[0], costh>=0)
            if target[1] != None:
                cond2 = reached(odo[1],target[1],sinth>=0)
            #print(target[0], " --x-- ", odo[0], "\n", target[1], "-------y------", odo[1])
            if cond1 and cond2:
            #if abs((target[0] - odo[0])) < eps[0] and abs((target[1] - odo[1])) < eps[1]:
                close = True
        return close

    def executeTrajectory(self):
        """
        Executes the saved trajectory (sequence of v,w and t)
        """
        period = 0.05
        for i in range(0, len(self.trajectory.targetPositions)):
            # target = [x,y,th]
            # pos = [x,y,th]
            v = self.trajectory.targetV[i]
            w = self.trajectory.targetW[i]
            self.setSpeed(v, w)
            end = False
            while not end:
                tIni = time.perf_counter()
                end = self.closeEnough(self.trajectory.targetPositions[i], w)
                if not end:
                    tFin = time.perf_counter()
                    time.sleep(period-(tFin-tIni))
        self.setSpeed(0, 0)

    ####################################################################################################
    # ODOMETRY FUNCTIONS

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

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=()) #additional_params?))
        self.p.start()
        print("PID: ", self.p.pid)

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        return self.x.value, self.y.value, self.th.value

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

            v, w = self.readSpeed(dT) # usar distancias de encoders directamente
            deltaTh = norm_pi(w*dT)
            eps = 0.01
            #print("----------\n", deltaTh)
            if -eps < deltaTh < eps:
                deltaSi = v*dT
            else:
                deltaSi = v/w*deltaTh
            #print("deltaSi: ", deltaSi)
            deltax = self.deltaX(deltaSi,deltaTh)
            deltay = self.deltaY(deltaSi,deltaTh)
            th = norm_pi(self.th.value+deltaTh)
            self.lock_odometry.acquire()
                # reducir SC (deltaX, etc)
            self.x.value += deltax
            self.y.value += deltay
            self.th.value = th
            self.lock_odometry.release()


            writeLog(self.f_log, [self.tLast-self.tInitialization, self.x.value, self.y.value, self.th.value, v, w, deltaTh, deltaSi])

            tEnd = time.perf_counter()
            time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" % (self.x.value, self.y.value, self.th.value))

    # Stop the odometry thread.
    def stopOdometry(self):
        """
        Stops odometry and sets the speed of all motors to 0
        """
        self.finished.value = True
        self.f_log.close()
        self.setSpeed(0,0)
        self.BP.set_motor_dps(self.motorGarras, 0)
        #self.BP.reset_all()

    ####################################################################################################
    # TRACKING FUNCTIONS

    def trackBall(self):
        """
        Tracks and catches the red ball
        """
        self.trackObject(self.ballArea, self.ballX, self.ballClawsArea, True, 5)

    def trackObject(self, targetSize, targetX = resolution[0]/2.0, targetClawsSize = 0, mustCatch = False, eps = 5):
        """
        Tracks an object with the given parameters, tries to catch it with the claws when its close enough.
        The robot spins at a constant rate while it doesnt detect the object
        """
        targetPositionReached = False
        # abrir garras, en paralelo:
        garrasAbiertas=True
        self.catch()
        # inicializar detector:
        detector = init_detector()
        period = 0.05

        vFin = self.vTarget / 2

        for img in self.cam.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):# todo: sleep como antes

            tIni = time.perf_counter()
            frame = img.array

            kp = search_blobs_detector(self.cam, frame, detector, verbose = False, show=False)
            self.rawCapture.truncate(0)


            if DEBUG_MODE and kp is None:
                print("No se donde esta la bola")

            if targetPositionReached:
                if self.closeEnough(objetivo, 0):
                    self.setSpeed(0, 0)
                    if mustCatch:
                        self.moveClaws()
                    break

                else:
                    #Decelerate
                    vFin=vFin/1.005
                    self.setSpeed(vFin,0)

            elif kp is None:
                # target not detected, spin in place to find it
                self.setSpeed(0,self.wTarget)
            else: # we see the target and the target position hasnt been reached
                d = horizontalDistance(kp, [targetX,0])
                A = kp.size
                w = getMappedW(self.wTarget, d, targetX - resolution[0], targetX)
                v = getMappedV(self.vTarget, A, targetSize)
                self.setSpeed(v,w)

                if targetSize-eps < A and not targetPositionReached:
                    # close enough to target, we set the objective from current position+some distance
                    targetPositionReached  = True
                    objetivo=self.advanceDistance(0.21)
                    objetivo=[objetivo[0], objetivo[1], None]
                    self.setSpeed(vFin/2,0)

            tEnd = time.perf_counter()
            #time.sleep(period-(tEnd-tIni))

    ####################################################################################################
    # CAMERA DEBUG FUNCTIONS

    def takePicture(self):
        #rawCapture = PiRGBArray(self.cam, size=(320, 240))

        now = datetime.datetime.now()
        self.cam.capture("photos/{:d}-{:d}-{:d}-{:02d}_{:02d}_{:02d}".format(now.year, now.month, now.day, now.hour, now.minute, now.second)+".png", format="png", use_video_port=True)
        #data = np.fromstring(stream.getvalue(), dtype=np.uint8)

        #cv2.imwrite("photos/{:d}-{:d}-{:d}-{:02d}_{:02d}".format(now.year, now.month, now.day, now.hour, now.minute)+".png", img.array)

    def detect_continuous(self):
        """
        Debug function to test camera and detector
        """
        period = 0.25 # 1 sec
        #rawCapture = PiRGBArray(self.cam, size=(320, 240))
        detector = init_detector()
        #cv2.namedWindow('frame', cv2.WINDOW_AUTOSIZE)
        time.sleep(1)
        self.cam.framerate=(1)
        for img in self.cam.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):

            tIni = time.perf_counter()
            frame = img.array

            cv2.imshow('frame', frame)
            #print(":(")

            noseque = search_blobs_detector(self.cam, frame, detector, verbose = True)
            self.rawCapture.truncate(0)


            tEnd = time.perf_counter()
            cv2.waitKey(int(1000*period - (tEnd-tIni)))

        cv2.destroyAllWindows()

    ####################################################################################################
    # CLAWS FUNCTIONS

    def catch(self):
        """
        Starts the process that closes or opens the claws
        """
        self.catcher = Process(target=self.moveClaws, args=()) #additional_params?))
        self.catcher.start()

        # decide the strategy to catch the ball once you have reached the target position
        pass

    def moveClaws(self):
        """
        Rutina paralela para cerrar las garras
        """
        self.lock_garras.acquire()

        DPS = 40 # 40� por segundo
        end = False
        period = 0.1

        self.finished.value = False
        while not self.finished.value and not end:
            tIni = time.perf_counter()
            if self.closing.value:
                end = self.BP.get_motor_encoder(self.motorGarras) >= 0
            else:
                end = self.BP.get_motor_encoder(self.motorGarras) <= -self.maxRotGarras
            if not end:
                self.BP.set_motor_dps(self.motorGarras, DPS if self.closing.value else -DPS)
                tEnd = time.perf_counter()
                time.sleep(period - (tEnd-tIni))

        self.BP.set_motor_dps(self.motorGarras, 0)
        self.closing.value= not self.closing.value
        self.lock_garras.release()


    def advanceDistance(self, dist):
        """
        Devuelve la posicion global (con respecto al origen de la odometria)
        resultante de avanzar dist desde la posicion actual.
        """
        xLoc=np.array([dist, 0, 0])
        xRW=np.array(self.readOdometry())
        xWorld=loc(np.dot(hom(xRW), hom(xLoc)))
        return xWorld

    ####################################################################################################
    # MAP & MOVING FUNCTIONS

    def recalculateGoal(self, odo, dX, dY, x_goal, y_goal, eps):
        if x_goal != None:
            dX = x_goal - odo[0]
        if y_goal != None:
            dY = y_goal - odo[1]
        if abs(dX) <= eps:
            x_goal = None
        if abs(dY) <= eps:
            y_goal = None
        th_goal = norm_pi(math.atan2(dY, dX))
        return x_goal, y_goal, th_goal

    def go(self, x_goal_ini, y_goal_ini, eps = 0.05):
        """
        Moves the robot to x_goal, y_goal (first it turns, then it advances, for cell navigation)
        returns True if it finds an obstacle
        """
        #xLoc=np.array([dist, 0, 0])
        #xRW=np.array(self.readOdometry())
        #xWorld = loc(np.dot(np.hom(xRW), hom(xLoc)))
        x_goal = x_goal_ini
        y_goal = y_goal_ini
        odo = self.readOdometry()
        period = 0.02
        #if sine is negative (if dY is negative) then the rotation must be negative
        w = self.wTarget /2
        dX = x_goal - odo[0]
        dY = y_goal - odo[1]
        th_goal = norm_pi(math.atan2(dY, dX))

        if (norm_pi(th_goal - odo[2]) < 0):
        #if xLoc[1] < 0:
            w = -w
            #th_goal = -th_goal
        print("X", dX, "Y", dY, "TH", th_goal, sep='\n')

        end = False

        while not end:
            tIni = time.perf_counter()
            odo = self.readOdometry()
            end = self.closeEnough([None, None, th_goal], w)
            if not end:
                self.setSpeed(0,w)
                tEnd = time.perf_counter()
                time.sleep(period - (tEnd - tIni))

        self.setSpeed(0,0)
        if self.detectObstacle(): # obstacle in front of the robot
            self.map.obstacleDetected(odo[0], odo[1], x_goal_ini, y_goal_ini)
            self.map.replanPath(odo[0], odo[1])
            # self.map.addObstacle(x_now, y_now, odo[2])
            # self.map.findPath(x_now, y_now, self.map.goal[0], self.map.goal[1])
            return True


        end = False
        v = self.vTarget
        self.setSpeed(v,0)
        end = False
        initial = np.array(self.readOdometry()[:-1])
        vmin = self.vTarget/6
        vmax = self.vTarget/1.5
        x_goal, y_goal, th_goal = self.recalculateGoal(odo, dX, dY, x_goal, y_goal, eps)
        while not end:
            tIni = time.perf_counter()
            odo = self.readOdometry()
            end = self.closeEnough([x_goal, y_goal, None], w)
            if not end:
                #if abs(th_goal - odo[2]) < eps:
                #    w = 0
                #elif th_goal > odo[2]:
                #    w = 0.1
                #elif th_goal < odo[2]:
                #    w = -0.1

                # speed:
                odo = np.array(self.readOdometry()[:-1])
                v = vInTrajectory(odo, initial,
                    np.array([x_goal_ini, y_goal_ini]), vmin, vmax)
                self.setSpeed(v,0)
                tEnd = time.perf_counter()
                time.sleep(period - (tEnd - tIni))
        self.setSpeed(0, 0)
        return False

    # ---------------------------------------------- p4:
    def setMap(self, map, ini=None, end=None):
        self.map = map
        # TODO: que pasa si ini!=[0,0]
        # pos = self.map.
        if ini is not None and end is not None:
            if ini[0]!=0 or ini[1]!=0:
                print('TODO......')
                exit(1)
            if not self.map.findPath(ini[0], ini[1],end[0],end[1]):
                print("ERROR en findPath")
                self.stopOdometry()

    def align(self, th_goal):
        period = 0.02
        w = self.wTarget / 2
        end = False
        self.setSpeed(0,w)
        while not end:
            tIni = time.perf_counter()
            end = self.closeEnough([None, None, th_goal], w)
            if not end:
                tEnd = time.perf_counter()
                time.sleep(period - (tEnd - tIni))
        self.setSpeed(0,0)

    def fixGoal(self,odo, goal, eps=0.02):
        for i in range(len(goal)-1):
            if -eps <= odo[i]-goal[i] <= eps:
                goal[i] = None
        return goal

    def goToNeighbour(self, neighbour):
        th_goal = math.pi/2
        if neighbour == 2:
            th_goal = 0
        elif neighbour == 4:
            th_goal = -math.pi/2
        elif neighbour == 6:
            th_goal = math.pi
        odo = self.readOdometry()
        dif = norm_pi(2*math.pi+th_goal-odo[2])
        w=self.wTarget
        if dif>0: # TODO: IGUAL AL REVES
            w=-w
        self.align(th_goal)
        goal = self.advanceDistance(self.map.sizeCell)
        goal=[goal[0], goal[1], None]
        goal = self.fixGoal(odo,goal)
        vFin = self.vTarget
        period = 0.02
        while True:
            tIni = time.perf_counter()
            if self.closeEnough(goal, 0):
                self.setSpeed(0, 0)
                break
            else:
                #Decelerate
                #vFin=vFin/1.005
                self.setSpeed(vFin,0)
                tEnd = time.perf_counter()
                time.sleep(period-(tEnd-tIni))

    def executePath_neigh(self):
        last = [0.,0.]
        for step in self.map.currentPath:
            neigh = self.map.neighbourFromCells(last,step)
            self.goToNeighbour(neigh)
            last = step

    def executePath(self):
        end = False
        while not end:
            for step in self.map.currentPath:
                # Go to next cell
                x, y = self.posFromCell(step[0], step[1])
                replan = self.go(x, y)
                if replan:
                    break
            end = not replan # end if there wasnt a replan




    def posFromCell(self, x,y):
        return x*self.map.sizeCell/1000.0, y*self.map.sizeCell/1000.0


    #def detectObstacle(self):

    #    obstacle,x,y=sensorDetection() #funcion que detecta obstaculo y por arte de magia te dice donde estan
    #    objectDetected(x,y)
    #    return obstacle



    def detectObstacle(self):
        dist=self.BP.get_sensor(self.portSensorUltrasonic)
        if dist<=self.map.sizeCell*self.min_celds:
            return True
        else
            return False
