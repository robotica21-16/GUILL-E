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


from geometry.geometry import *

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

from p3.color_blobs import *

resolution=[320,240]


def reached(x, target, greater):
    if greater:
        return x>=target
    else:
        return x<=target

def reachedAngle(th, target, w):
    if w > 0:
        if target < 0 and th > 0:
            return th >= (2*math.pi + target)
        else:
            return th >= target
    elif w < 0:
        if target > 0 and th < 0:
            return (2*math.pi+th) <= target
        else:
            return th <= target

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

        self.offset_right = 0.9995 # The way the bot is built, the left tire spins slightly slower than right

        # Camera Initialization
        #self.cam = picamera.PiCamera()
        #self.cam.resolution = (320, 240)
        #self.cam.resolution = (640, 480)
        #self.cam.framerate = 32
        #self.cam.rotation = 180
        #####################
        
        
        self.cam = picamera.PiCamera()

        #self.cam.resolution = (320, 240)
        self.cam.resolution = tuple(resolution)
        self.cam.framerate = 60
        #self.rawCapture = PiRGBArray(self.cam, size=(320, 240))
        self.rawCapture = PiRGBArray(self.cam, size=tuple(resolution))
        
        self.cam.rotation=180
        

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        self.wmax = math.pi/3
        self.vmax = 1.0/4.0
        self.vTarget = self.vmax
        self.wTarget = self.wmax/2


        self.targetArea = 120#200.71975708007812 # TODO: poner bien!!!!!
        self.targetGarras = 60
        self.targetX = resolution[0]/2.0 # 318.3089599609375 
        self.lock_garras = Lock()
        
        self.closing = Value('b',0)
        # Configure sensors, for example a touch sensor.
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        self.ruedaIzq = self.BP.PORT_D
        self.ruedaDcha = self.BP.PORT_A
        self.motorGarras = self.BP.PORT_C
        self.maxRotGarras = 75 # angulo de giro de las garras 30 grados, comprobar!
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

        # odometry update period --> UPDATE value!
        self.P = 0.05

        now = datetime.datetime.now()
        self.f_log = open("logs/{:d}-{:d}-{:d}-{:02d}_{:02d}".format(now.year, now.month, now.day, now.hour, now.minute)+"-log.txt","a")#append
        fila = ["t", "x", "y", "th", "v*100", "w", "dTh", "dSi"]
        self.f_log.write("\t".join([str(e) for e in fila]) + "\n")
        time.sleep(0.2)

    def setSpeed(self, v, w):
        """
        Sets the speed as degrees PS to each motor according to
        v (linear ms) and w (angular rad s)
        """
        # wID = [wI, wD]:
        if v == 0 and False:
            vI = w*self.eje_rueda
            vD = -vI
            wI = vI*(self.R_rueda*2.0*math.pi)
            wD = vD*(self.R_rueda*2.0*math.pi)
        else:
            wDI = izqDchaFromVW(self.R_rueda, self.L, v, w)
            wD = wDI[0]
            wI = wDI[1]
        speedDPS_left = wI/math.pi*180
        speedDPS_right = self.offset_right*wD/math.pi*180
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


    def closeEnough(self, target, w, eps=np.array([0.02, 0.02, 0.2])):
        odo = self.readOdometry()
        close = False
        if target[0] == None and target[1] == None:
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
                cond1 = reached(odo[0],target[0], costh>0)
            elif target[1] != None:
                cond2 = reached(odo[1],target[1],sinth>0)
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
        #rint("wi, wd: ", wI, wD)
        v,w = vWiFromIzqDcha(self.R_rueda, self.L, wI, wD)
        #print("v, w: ", v, w)
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
            

            self.writeLog(v,w, deltaTh, deltaSi)

            tEnd = time.perf_counter()
            time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))


    def targetW(self, d, dmin=-resolution[0]/2.0, dmax=resolution[0]/2.0, eps=15):
        if (abs(d)<eps):
            return 0
        return np.interp(d, [dmin, dmax], [-self.wTarget, self.wTarget])

    def targetV(self, A):
        """
        ...
        """
        # cuando A=0 (en el infinito) -> targetArea-A = targetArea -> v=vmax
        # cuando A=a (en el objetivo) -> targetArea-A = 0 -> v = 0
        #print(A, "A, target", self.targetArea, "Resta: ", self.targetArea-A)
        return self.vTarget-np.interp(A-self.targetArea, [-self.targetArea,0], [0, self.vTarget-0.1])

    def trackObject(self, view=False, colorRangeMin=[0,0,0], colorRangeMax=[255,255,255]):
        # targetSize=??, target??=??, catch=??, ...)
        # targetFound = False
        targetPositionReached = False
        finished = False
        garrasAbiertas=False
        cerrando=False
        detector = init_detector()
        period = 0.05
        #cv2.namedWindow('frame', cv2.WINDOW_AUTOSIZE)
        #self.cam.framerate=(1)
        print("Estoy vivo")
        a=30
        vFin = self.vTarget/2
        tRecorridoFinal = 0.06/vFin
        for img in self.cam.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):

            tIni = time.perf_counter()
            frame = img.array
            
            #cv2.imshow('frame', frame)
            #print(":(")
            
            kp = search_blobs_detector(self.cam, frame, detector, verbose = False, show=False)
            self.rawCapture.truncate(0)
            
            if kp is None:
                print("No se donde esta la bola")
                
                self.setSpeed(0,0)
                
            else:
                # 1. search the most promising blob ..
                #kp = search_blobs(self.cam, view)
                #while not targetPositionReached:
                # 2. decide v and w for the robot to get closer to target position

                d = horizontalDistance(kp, [self.targetX,0])
                #r = kp.size/2 # suponiendo que es el diametro
                #A = r**2 * math.pi
                A=kp.size
                w = self.targetW(d)
                v = self.targetV(A)
                #print(v,w, A,d)
                self.setSpeed(v,w)
                eps = 15
                a-=1
                if a<=0:
                    a=30
                    print(A)
                    
                    
                    
                #if self.targetGarras -eps < A < self.targetGarras + eps:
                if self.targetGarras < A and not self.closing.value and not garrasAbiertas:
                    garrasAbiertas=True
                    self.catch()
                    #abrir las garras 
                
                if self.targetArea-eps < A < self.targetArea+eps:
                    targetPositionReached  = True
                    
                    self.setSpeed(0,0)
                    print("Estoy delante")
                    #return finished
            
            if targetPositionReached:
                if tRecorridoFinal<=0:
                    self.setSpeed(0,0)
                
                    if not cerrando:
                        cerrando=True
                        self.catch()
                else:
                    self.setSpeed(vFin,0)
                    tRecorridoFinal-=period
                    
            tEnd = time.perf_counter()
            
            cv2.waitKey(int(1000*period - (tEnd-tIni)))
                    
    def takePicture(self):
        #rawCapture = PiRGBArray(self.cam, size=(320, 240))
        
        now = datetime.datetime.now()
        self.cam.capture("photos/{:d}-{:d}-{:d}-{:02d}_{:02d}_{:02d}".format(now.year, now.month, now.day, now.hour, now.minute, now.second)+".png", format="png", use_video_port=True)
        #data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        
        #cv2.imwrite("photos/{:d}-{:d}-{:d}-{:02d}_{:02d}".format(now.year, now.month, now.day, now.hour, now.minute)+".png", img.array)

    def detect_continuous(self):
        period = 0.25 # 1 sec
        #rawCapture = PiRGBArray(self.cam, size=(320, 240))
        detector = init_detector()
        #cv2.namedWindow('frame', cv2.WINDOW_AUTOSIZE)
        time.sleep(1)
        self.cam.framerate=(1)
        for img in self.cam.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):

            tIni = time.perf_counter()
            frame = img.array
            
            #cv2.imshow('frame', frame)
            #print(":(")
            
            noseque = search_blobs_detector(self.cam, frame, detector, verbose = True)
            self.rawCapture.truncate(0)
            
            
            tEnd = time.perf_counter()
            cv2.waitKey(int(1000*period - (tEnd-tIni)))
            
        cv2.destroyAllWindows()
            


    def catch(self):
        self.catcher = Process(target=self.moveClaws, args=()) #additional_params?))
        self.catcher.start()

        # decide the strategy to catch the ball once you have reached the target position
        pass

    def moveClaws(self):
        """
        Rutina paralela para cerrar las garras
        """
        self.lock_garras.acquire()

        DPS = 30 # 20º por segundo
        end = False
        period = 0.2
        print("hoho")
        self.finished.value = False
        while not self.finished.value and not end:
            print("hehe")
            if self.closing.value:
                print("Todo ok")
            else:
                print("Todo mal")
            tIni = time.perf_counter()
            if self.closing.value:
                end = self.BP.get_motor_encoder(self.motorGarras) > 0
            else:
                print(self.BP.get_motor_encoder(self.motorGarras))
                end = self.BP.get_motor_encoder(self.motorGarras) < -self.maxRotGarras
            if not end:
                self.BP.set_motor_dps(self.motorGarras, DPS if self.closing.value else -DPS)
                tEnd = time.perf_counter()
                time.sleep(period - (tEnd-tIni))
        print("huhu")
        self.BP.set_motor_dps(self.motorGarras, 0)
        self.closing.value= not self.closing.value
        self.lock_garras.release()

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
