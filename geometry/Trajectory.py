#!/usr/bin/python
# -*- coding: UTF-8 -*-

import time     # import the time library for the sleep function
import sys

from geometry.geometry import *
from geometry.utilsbot import animarBot

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

class Movement:
    """
    Un movimiento circular definido por vc=[v,w] y t
    """
    def __init__(self, vc=[0,0], t=0):
        self.vc = vc
        self.t = t

class Trajectory:
    """
    Secuencia de movimientos [vc, t], donde vc=[v,w]
    """
    def __init__(self, movements=[], wxr=[0,0,0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """
        self.movements = movements
        self.wxr = wxr


    def addMove(self, vc, t):
        self.movements += [Movement(vc, t)]

    # def addMove(self, movement: Movement):
    #     self.movements += [movement]

    def draw(self, fps=24):
        for move in self.movements:
            self.wxr,_ = animarBot(self.wxr, move.vc, move.t, fps)
