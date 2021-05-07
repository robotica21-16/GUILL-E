#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''


import brickpi3 # import the BrickPi3 drivers

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

from trabajo.sample_matching import match_images

DEBUG_MODE=False # show less prints
resolution=[320,240] # camera resolution
black=2500 # floor sensor black threshold
white=1970 # floor sensor white threshold

class Robot:
    """
    All units are in the international system unless specified (and angles in rad)
    """
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params.
        Initialize Motors and Sensors according to the set up in GUILLE
        """
        ##################################################
        # Robot construction parameters

        self.R_rueda = 0.027
        self.L = 0.140
        self.eje_rueda = self.L/2.0
        self.len_color_eje = 0.11 # longitud del detector de color al eje

        self.offset_right = 1.0#0.9995 # The way the bot is built, the left tire spins slightly slower than right

        ##################################################
        # Camera initialization
        self.cam = picamera.PiCamera()

        self.cam.resolution = tuple(resolution)
        self.cam.framerate = 60
        self.rawCapture = PiRGBArray(self.cam, size=tuple(resolution))

        self.cam.rotation=180

        ##################################################
        self.BP = brickpi3.BrickPi3()

        ##################################################
        # Speed parameters
        self.wmax = math.pi/3
        self.vmax = 1.0/4.0
        self.vTarget = self.vmax
        self.wTarget = self.wmax * 0.4 #/2

        self.rotIzqDeg = 0
        self.rotDchaDeg = 0

        self.useGyro = True

        ##################################################
        # Ball parameters
        self.ballArea = 65
        self.ballClawsArea = 60
        self.ballX = resolution[0]/2.0
        self.ballDistance = 0.36


        ##################################################
        # Sensors and motors
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

        #################################################
        ##Adjust Claws to be closed
        posClawsIni = self.BP.get_motor_encoder(self.motorGarras)
        if(posClawsIni < -5 or posClawsIni > 0):
            print("Adjusting claws")
            while(posClawsIni < -5):
                self.BP.set_motor_dps(self.motorGarras, 20)
                posClawsIni = self.BP.get_motor_encoder(self.motorGarras)
            while(posClawsIni > 0):
                self.BP.set_motor_dps(self.motorGarras, -20)
                posClawsIni = self.BP.get_motor_encoder(self.motorGarras)
        self.BP.set_motor_dps(self.motorGarras, 0)


        #################################################
        # ultrasonic
        self.portSensorUltrasonic = self.BP.PORT_4
        self.BP.set_sensor_type(self.portSensorUltrasonic, self.BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)
        self.min_cells=1  # cm

        #################
        # light sensor
        self.portSensorLight = self.BP.PORT_1
        self.BP.set_sensor_type(self.portSensorLight, self.BP.SENSOR_TYPE.NXT_LIGHT_ON)

        #################
        # gyro:
        self.portGyro= self.BP.PORT_2
        self.BP.set_sensor_type(self.portGyro, self.BP.SENSOR_TYPE.EV3_GYRO_ABS)

        ####################################################################################################
        # odometry shared memory values
        # usar parametro
        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.th = Value('d',0.0)
        self.th_abs = Value('d',0.0) # absoluto, solo se debe usar para restar dos valores
        self.finished = Value('b',1) # boolean to show if odometry updates are finished
        self.changeClaws = Value('b',1)

        self.tLast = time.perf_counter()
        self.tInitialization = self.tLast

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        self.detectorLock = Lock()
        # odometry update period
        self.P = 0.05

        ####################################################################################################
        # Logs
        now = datetime.datetime.now()
        self.f_log = open("logs/{:d}-{:d}-{:d}-{:02d}_{:02d}".format(now.year, now.month, now.day, now.hour, now.minute)+"-log.txt","a")#append
        fila = ["t", "x", "y", "th", "v", "w", "dTh", "dSi"]
        self.f_log.write("\t".join([str(e) for e in fila]) + "\n")
        self.f_log.flush()

        ####################################################################################################
        ### Homographies:
        self.templateR2D2 = cv2.imread("trabajo/R2-D2_s.png", cv2.IMREAD_COLOR)
        self.templateBB8 = cv2.imread("trabajo/BB8_s.png", cv2.IMREAD_COLOR)

        time.sleep(5)

    ####################################################################################################
    # SPEED FUNCTIONS
    def setSpeed(self, v, w):
        """
        Sets the speed as degrees PS to each motor according to
        v (linear ms) and w (angular rad s)
        """
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
        try:
            izq = self.BP.get_motor_encoder(self.ruedaIzq)
            dcha = self.BP.get_motor_encoder(self.ruedaDcha)
            izq_bk = izq
            dcha_bk = dcha
            izq = izq-self.rotIzqDeg
            dcha = dcha-self.rotDchaDeg

            self.rotIzqDeg = izq_bk
            self.rotDchaDeg = dcha_bk
            return izq/dT, dcha/dT
        except IOError as error:
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
        """
        In: target [x, y, th] (any value can be None),
            w (angular speed)
        Out: True if the target has been reached according to the odometry.
        Only non-None values are checked
        """
        odo = self.readOdometry()
        close = False
        if target[0] == None and target[1] == None and target[2] == None:
            close = True
        elif target[0] == None and target[1] == None: # only angle
            if self.useGyro: # test according to gyro
                return reachedAngleGyro(np.radians(self.angleGyro() - self.turnedGyro), target[2], w)
            else: # test with odo
                return reachedAngle(odo[2], target[2], w)
        else: # check positions
            sinth = np.sin(odo[2])
            costh = np.cos(odo[2])
            cond1 = True
            cond2 = True
            if target[0] != None:
                cond1 = reached(odo[0],target[0], costh>=0)
            if target[1] != None:
                cond2 = reached(odo[1],target[1],sinth>=0)
            if cond1 and cond2:
                close = True
        return close

    def setTurnedGyro(self):
        """
        Save current gyro angle for future comparisons
        """
        self.turnedGyro = self.angleGyro()

    def executeTrajectory(self):
        """
        Executes the saved trajectory (sequence of v,w and t)
        Time-based version
        """
        period = 0.05
        for i in range(len(self.trajectory.targetPositions)):
            print("Paso:", i, "----------------------------")
            self.setTurnedGyro()
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

            v, w = self.readSpeed(dT) # usar distancias de encoders directamente
            deltaTh = norm_pi(w*dT)
            eps = 0.01
            if -eps < deltaTh < eps:
                deltaSi = v*dT
            else:
                deltaSi = v/w*deltaTh
            #print("deltaSi: ", deltaSi)
            deltax = self.deltaX(deltaSi,deltaTh)
            deltay = self.deltaY(deltaSi,deltaTh)
            th_abs = self.th_abs.value+deltaTh
            th = norm_pi(self.th.value+deltaTh)
            self.lock_odometry.acquire()
                # reducir SC (deltaX, etc)
            self.x.value += deltax
            self.y.value += deltay
            self.th.value = th
            self.th_abs.value = th_abs # only used for some comparisons
            self.lock_odometry.release()
            # write a row to the log:
            writeLog(self.f_log, [self.tLast-self.tInitialization, self.x.value, self.y.value, self.th.value, v, w, deltaTh, deltaSi])

            tEnd = time.perf_counter()
            time.sleep(max(self.P - (tEnd-tIni),0))

        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" % (self.x.value, self.y.value, self.th.value))

    def setOdometry(self, value):
        """
        Sets the odometry to any value [x,y,th] (in m,m,rad)
        """
        self.lock_odometry.acquire()
        self.finished.value = True
        self.x.value = value[0]
        self.y.value = value[1]
        self.th.value = value[2]
        self.finished.value = False
        self.lock_odometry.release()

    # Stop the odometry thread.
    def stopOdometry(self):
        """
        Stops odometry and sets the speed of all motors to 0
        """
        self.finished.value = True
        self.f_log.close()
        self.setSpeed(0,0)
        self.BP.set_motor_dps(self.motorGarras, 0)
        self.BP.reset_all()

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
                print("reaching objetivo", objetivo, "odo:", self.readOdometry())
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
                self.setSpeed(0,self.wTarget * 0.6)
            else: # we see the target and the target position hasnt been reached
                d = horizontalDistance(kp, [targetX,0])
                A = kp.size

                w = getMappedW(self.wTarget, d, targetX - resolution[0], targetX)
                v = getMappedV(self.vTarget/2.0, A, targetSize)
                self.setSpeed(v,w)
                if DEBUG_MODE:
                    print("A:",A, "d:",d)
                if targetSize-eps < A and not targetPositionReached:
                    # close enough to target, we set the objective from current position+some distance
                    targetPositionReached  = True
                    objetivo=self.advanceDistance(self.ballDistance)
                    objetivo=[objetivo[0], objetivo[1], None]
                    self.setSpeed(vFin/2,0)
                    if DEBUG_MODE:
                        print("target pos reached, advancing to ", objetivo)

            tEnd = time.perf_counter()
            #time.sleep(period-(tEnd-tIni))

    ####################################################################################################
    # CAMERA DEBUG FUNCTIONS
    def takePicture(self):
        """
        Take a picture and save it in photos directory (name based on current time)
        """
        now = datetime.datetime.now()
        self.cam.capture("photos/{:d}-{:d}-{:d}-{:02d}_{:02d}_{:02d}".format(now.year, now.month, now.day, now.hour, now.minute, now.second)+".png", format="png", use_video_port=True)

    def detect_continuous(self):
        """
        Debug function to test camera and cv2 detector
        """
        period = 0.25 # 1 sec
        detector = init_detector()
        time.sleep(1)
        self.cam.framerate=(1)
        for img in self.cam.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            tIni = time.perf_counter()
            frame = img.array
            cv2.imshow('frame', frame)
            noseque = search_blobs_detector(self.cam, frame, detector, verbose = True)
            self.rawCapture.truncate(0)
            tEnd = time.perf_counter()
            cv2.waitKey(int(1000*period - (tEnd-tIni)))
        cv2.destroyAllWindows()



    ####################################################################################################
    # Floor light detector functions

    def waitForWhite(self, coordinate, value):
        """
        Updates the coordinate when it finds a white line (0->x, 1->y)
        Both parameters are lists (must be of the same length),
        and the process will be queued for each value
        """
        white=[True]*len(coordinate)
        print("coord ", coordinate, value)
        self.lineDetectorProcess = Process(target = self.lineDetector, args=(white, coordinate, value))
        self.lineDetectorProcess.start()



    def updateCoordValue(self,coordinate, value):
        """
        update the coordinate (0->x, 1->y) with the given value, taking into account
        the current theta from the odometry
        """
        if coordinate == 0: # x
            self.lock_odometry.acquire()
            self.x.value = value + self.len_color_eje * np.cos(self.readOdometry()[2])
            self.lock_odometry.release()
        else: # y
            self.lock_odometry.acquire()
            self.y.value = value + self.len_color_eje * np.sin(self.readOdometry()[2])
            self.lock_odometry.release()

    def lineDetector(self, whites,coordinates, values):
        """
        For each white, coordinate, and value in the three parameters (lists),
        updates the given coordinate with the given value when a line of the
        given color (white->white, not white->black) is detected.
        It has to detect the same color twice in a row for consistency.
        """
        period = 0.1
        for i in range(len(whites)):
            white = whites[i]
            coordinate = coordinates[i]
            value=values[i]
            end = False
            nDetected = 0
            while not self.finished.value and not end:
                tIni = time.perf_counter()
                if white: # looking for a white stripe
                    if self.colorSensorWhite():
                        nDetected+=1 # count the detection
                    else:
                        nDetected=0 # reset
                    if nDetected>1: # we detected two in a row
                        print("WHITE DETECTED")
                        self.updateCoordValue(coordinate, value) # update
                        end = True
                else: # looking for a black stripe
                    if self.colorSensorBlack():
                        nDetected+=1 # count the detection
                    else:
                        nDetected=0 # reset
                    if nDetected>1: # we detected two in a row
                        print("BLACK DETECTED")
                        self.updateCoordValue(coordinate, value) # update
                        end = True
                tEnd = time.perf_counter()
                time.sleep(period - (tEnd-tIni))
            time.sleep(1) # sleep one second to prevent the second line being detected too soon


    ####################################################################################################
    # CLAWS FUNCTIONS
    def catch(self):
        """
        Starts the process that closes or opens the claws
        """
        self.catcher = Process(target=self.moveClaws, args=()) #additional_params?))
        self.catcher.start()

    def moveClaws(self):
        """
        Rutina paralela para cerrar las garras
        """
        self.lock_garras.acquire()
        DPS = 40 # 40º por segundo
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

    def fixOdometryFromObstacle(self, neighbour, x_objective, y_objective):
        """
        Unused function, for updating odometry based on obstacles
        """
        print("actualizando odo:", self.readOdometry())
        plusone=False
        if self.dist / 100 >= 0.95 * self.mapa.sizeCell/1000:
            plusone=True
        self.lock_odometry.acquire()
        if neighbour == 0:
            if plusone:
                y_objective+=self.mapa.sizeCell/1000
            self.y.value = y_objective - (self.mapa.sizeCell /1000 / 2 + self.dist / 100)
        elif neighbour == 4:
            if plusone:
                y_objective-=self.mapa.sizeCell/1000
            self.y.value = y_objective + (self.mapa.sizeCell /1000/ 2 + self.dist / 100)
        elif neighbour == 2:

            if plusone:
                x_objective+=self.mapa.sizeCell/1000
            self.x.value = x_objective - (self.mapa.sizeCell /1000/ 2 + self.dist / 100)
        elif neighbour == 6:

            if plusone:
                x_objective-=self.mapa.sizeCell/1000
            self.x.value = x_objective + (self.mapa.sizeCell /1000 / 2 + self.dist / 100)
        self.lock_odometry.release()

        print("nuevos valores odo:", self.readOdometry())

    def rel_angle(self, dX, dY, th):
        """
        Returns the relative angle between th and the (dX, dY) vector
        """
        th_abs = math.atan2(dY, dX) # -pi a pi
        #neighbour = self.mapa.neighbou
        return norm_pi(th_abs - th)


    def go(self, x_goal_ini, y_goal_ini, eps = 0.05, checkObstacles=True, checkObstaclesMoving=False):
        """
        Moves the robot to x_goal, y_goal (first it turns, then it advances, for cell navigation)
        returns True if it finds an obstacle
        if checkObstaclesMoving, it detects the obstacles while advancing (disabled: does not work very well)
        """

        x_goal = x_goal_ini
        y_goal = y_goal_ini
        odo_ini = self.readOdometry()
        odo = odo_ini
        period = 0.02
        #if sine is negative (if dY is negative) then the rotation must be negative
        w = self.wTarget
        dX = x_goal - odo[0]
        dY = y_goal - odo[1]
        if not self.useGyro: # odometry based turn
            th_goal = norm_pi(math.atan2(dY, dX))
            if (norm_pi(th_goal - odo[2]) < 0):
                w = -w
        else: # gyro based turn
            th_goal = self.rel_angle(dX, dY, odo[2])
            if (-math.pi/16 < th_goal < math.pi/16):
                th_goal = 0
            if (norm_pi(th_goal) < 0):
                w = -w
        end = False
        self.setTurnedGyro()
        while not end: # turn
            tIni = time.perf_counter()
            odo = self.readOdometry()
            end = self.closeEnough([None, None, th_goal], w)
            if not end:
                self.setSpeed(0,w)
                tEnd = time.perf_counter()
                time.sleep(max(period - (tEnd - tIni),0))

        self.setSpeed(0,0)
        # obstaculos:
        if checkObstacles and self.detectObstacle(): # obstacle in front of the robot
            self.mapa.obstacleDetected(odo[0], odo[1], x_goal, y_goal)
            if not self.mapa.replanPath(odo[0], odo[1]):
                print("Unable to find a path")
                self.stopOdometry()
                exit(0)
            return True
        end = False
        v = self.vTarget
        self.setSpeed(v,0)
        initial = np.array(self.readOdometry()[:-1])
        vmin = self.vTarget/4
        vmax = self.vTarget/1.5
        x_goal, y_goal, th_goal = self.recalculateGoal(odo, dX, dY, x_goal, y_goal, eps)
        while not end: # until we reach the goal
            tIni = time.perf_counter()
            odo = self.readOdometry()
            end = self.closeEnough([x_goal, y_goal, None], w)
            if not end:
                # only if checkObstaclesMoving:
                if checkObstaclesMoving and self.detectObstacle(cell_proportion=0.8): # obstacle in front of the robot
                    neighbour = self.mapa.obstacleDetected(odo[0], odo[1], x_goal_ini, y_goal_ini)
                    self.fixOdometryFromObstacle(neighbour, x_goal_ini, y_goal_ini)
                    if not self.mapa.replanPath(odo[0], odo[1]):
                        print("Unable to find a path")
                        self.stopOdometry()
                        exit(0)
                    return True
                # slow down based on distance to goal:
                odo = np.array(self.readOdometry()[:-1])
                v = vInTrajectory(odo, initial,
                    np.array([x_goal_ini, y_goal_ini]), vmin, vmax) # variable speed
                self.setSpeed(v,0)
                tEnd = time.perf_counter()
                time.sleep(max(period - (tEnd - tIni),0))
        # in the end, stop:
        self.setSpeed(0, 0)
        return False

    # ---------------------------------------------- p4:
    def setMapNoPath(self, mapa):
        """
        sets the map and the initial positions for the odometry (ini, end in cells)
        Finds the shortest path from ini to end
        """
        self.mapa = mapa


    def setMap(self, mapa, ini=None, end=None):
        """
        sets the map and the initial positions for the odometry (ini, end in cells)
        Finds the shortest path from ini to end
        """
        self.mapa = mapa

        if ini is not None:
            x, y = self.posFromCell(ini[0], ini[1])
            self.setOdometry([x, y, ini[2]])

        if end is not None:
            if not self.mapa.findPath(ini[0], ini[1],end[0],end[1]):
                print("ERROR en findPath")
                self.stopOdometry()

    def setPath(self, ini, end):
        """
        Pre: a map has to have been set
        Set a path from ini to end (both in m, [x,y])
        """
        if not self.mapa.findPath(ini[0], ini[1],end[0],end[1]):
            print("ERROR en findPath")
            self.stopOdometry()


    def setPathFromCurrentPosition(self, end):
        """
        Pre: a map has to have been set
        Set a path to end (in m) from the current odometry position
        """
        odo = self.readOdometry()
        if not self.mapa.findPathFromPos(odo[0], odo[1],end[0],end[1]):
            print("ERROR en findPath")
            self.stopOdometry()

    def executePath(self, debug=False, checkObstacles=True):
        """
        Executes the path in the current map, moving from cell to cell
        """
        end = False
        while not end:
            replan = False
            print(self.mapa.currentPath)
            for step in self.mapa.currentPath:
                # Go to next cell
                x, y = self.posFromCell(step[0], step[1])
                replan = self.go(x, y, checkObstacles=checkObstacles)
                if replan:
                    if debug:
                        self.mapa.drawMap(saveSnapshot=False)
                    break
            end = not replan # end if there wasnt a replan

    def detectHomography(self, DEBUG=0, verbose=False):
        """
        takes a picture and  returns an int based on the detection:
        0 -> nothing
        1 -> r2
        2 -> el otro
        """
        with picamera.array.PiRGBArray(self.cam) as stream:
            self.cam.capture(stream, format='bgr')
            # At this point the image is available as stream.array
            image = stream.array
            if match_images(self.templateR2D2, image, DEBUG=DEBUG, verbose=verbose):
                return 1
            elif match_images(self.templateBB8, image, DEBUG=DEBUG, verbose=verbose):
                return 2
            else:
                return 0

    def detectR2D2(self, DEBUG=0, verbose=False):
        """
        takes a picture and checks if R2D2 is there
        """
        with picamera.array.PiRGBArray(self.cam) as stream:
            self.cam.capture(stream, format='bgr')
            image = stream.array
            return match_images(self.templateR2D2, image, DEBUG=DEBUG, verbose=verbose)

    def detectBB8(self, DEBUG=0, verbose=False):
        """
        takes a picture and checks if BB8 is there
        """
        with picamera.array.PiRGBArray(self.cam) as stream:
            self.cam.capture(stream, format='bgr')
            image = stream.array
            return match_images(self.templateBB8_s, image, DEBUG=DEBUG, verbose=verbose)

    def posFromCell(self, x,y):
        """
        Returns the real position (in m) from the given cell coordinates
        (center of the cell)
        """
        return (x+0.5)*self.mapa.sizeCell/1000.0, (y+0.5)*self.mapa.sizeCell/1000.0


    def detectObstacle(self, cell_proportion=1.0):
        """
        Returns true if the ultrasonic sensor returns a distance less than the size of a cell
        False otherwise (or if there is a sensor error)
        """
        try:
            self.dist=self.BP.get_sensor(self.portSensorUltrasonic)
            return self.dist<=self.mapa.sizeCell/10.0*self.min_cells*cell_proportion
        except brickpi3.SensorError as error:
            print(error)
            return False

    def testDistance(self, period=0.5):
        """
        Test ultrasonic sensor. Prints the measured distance
        """
        while True:
            dist = self.BP.get_sensor(self.portSensorUltrasonic)
            print("dist: ", dist)
            time.sleep(period)

    def colorSensorValue(self):
        """
        Returns the light sensor value
        """
        return self.BP.get_sensor(self.portSensorLight)

    def colorSensorBlack(self):
        """
        Returns true if black is detected according to the threshold
        """
        return self.colorSensorValue()>=black

    def colorSensorWhite(self):
        """
        Returns true if white is detected according to the threshold
        """
        return self.colorSensorValue()<=white


    def angleGyro(self):
        """
        absolute angle turned In degrees
        They use the opposite angles...
        """
        return -self.BP.get_sensor(self.portGyro)


    def relocateWithSonar(self, angle, relocationPosition, distance1 = 35, distance2 = 25, eps = 0.2):
        """
        Updates the odometry using a wall.
        - angle is the angle in which it will look for the wall and put itself perpendicular to.
        - relocationPosition [x,y,th] is the position it will be updated to (any can be None)
        - distance1 (cm) is the distance from the wall the robot will get to before fixing the odometry
        - distance2 (cm) is the final distance to the wall after the odometry fix
        - eps (cm) is used for the minimum distance detection
        After the odometry fix, it also sets the Y coordinate based on the detected angle error
        """
        w = self.wTarget
        sleepTime = 0.4
        minVal = math.inf
        diff = []
        prevDist = math.inf
        period = 0.05
        if (norm_pi(angle - self.readOdometry()[2]) < 0):
            w = -w
        end = False
        odo_ini = self.readOdometry()
        # face <angle> before moving forward
        self.setSpeed(0,w)
        while not end:
            tIni = time.perf_counter()
            end = self.closeEnough([None, None, angle], w)
            if not end:
                tFin = time.perf_counter()
                time.sleep(period-(tFin-tIni))
        self.setSpeed(0,0)
        w = self.wTarget*0.4
        # oriented to <angle>, move forward until distance1:
        while not self.detectObstacle() or self.dist > distance1:
            tIni = time.perf_counter()
            self.setSpeed(self.vTarget / 2, 0)
            tFin = time.perf_counter()
            time.sleep(period-(tFin-tIni))
            print(self.dist)
        ## End of lineal movement
        self.setSpeed(0,0)
        time.sleep(1)
        # detect which way it has to turn (detect the gradient):
        while prevDist == self.dist or prevDist == math.inf:
            print("self.dist", self.dist)
            self.setSpeed(0,w)
            time.sleep(0.5)
            detected = self.detectObstacle()
            if prevDist != math.inf:
                if self.dist > prevDist:
                    print("end", self.dist, prevDist)
                    w = -w # turn the other way
                    time.sleep(0.5)
                if prevDist != self.dist:
                    break # keep turning the same way
            prevDist = self.dist
        self.setSpeed(0,0)

        # save current th and traveled distance to fix y later:
        odo = self.readOdometry()
        r = distance(np.array(odo_ini[:1]), np.array(odo[:1]))
        th_before = self.th_abs.value

        time.sleep(1)
        prevDist = math.inf
        w /= 2
        self.setSpeed(0,w)
        # now we turn expecting to see the distance decrease. When it gets greater,
        # we know we have reached the minimum, which is the angle perpendicular to the wall
        while True:
            detected = self.detectObstacle()
            if prevDist< minVal:
                minVal = prevDist
                thMin = self.th_abs.value # save for Y
            if prevDist != self.dist:
                if (not detected) or self.dist > distance1:
                    diff = []
                    prevDist = math.inf
                elif self.dist != prevDist:
                    if w < 0:
                        end = self.dist >= minVal + eps
                    else:
                        end = self.dist >= minVal + eps
                    if end and detected:
                        break
                    prevDist = self.dist
                time.sleep(0.5)

        detected = False
        end = False

        # correct y according to the measured theta error:
        th_abs_now = self.th_abs.value
        dTh = th_abs_now - th_before
        dY = np.sin(dTh) * r
        # correct th:
        odo = self.readOdometry()
        dTh = th_abs_now-thMin

        self.setOdometry([odo[0], odo[1] + dY, odo[2]+dTh])

        # we are perpendicular, get to distance2 of the wall:
        while not self.detectObstacle() or self.dist > distance2:
            tIni = time.perf_counter()
            self.setSpeed(self.vTarget / 2, 0)
            tFin = time.perf_counter()
            time.sleep(period-(tFin-tIni))

        odo = self.readOdometry()
        # update coordinate based on <relocationPosition>:
        for i, p in enumerate(relocationPosition):
            if p == None:
                relocationPosition[i] = odo[i]

        self.setOdometry(relocationPosition)
