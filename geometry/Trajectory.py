#!/usr/bin/python
# -*- coding: UTF-8 -*-

import time     # import the time library for the sleep function
import sys

from geometry.geometry import *
from geometry.utilsbot import animarBot, simubot

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

class Movement:
    """
    Un movimiento circular definido por vc=[v,w] y t
    """
    def __init__(self, vc=[0,0], t=0):
        self.vc = vc
        self.t = t

    def __str__(self):
        return str(self.t) + ": " + str(self.vc)

class Trajectory:
    """
    Secuencia de movimientos [vc, t], donde vc=[v,w]
    """
    def __init__(self, movements=[], wxr=[0,0,0]):
        """
        Initialize the trajectory as a sequence of movements and an initial position
        """
        self.movements = movements
        self.wxr = wxr



    def addMove(self, vc, t):
        """
        Adds the move defined by vc and t
        """
        self.movements += [Movement(vc, t)]

    # def addMove(self, movement: Movement):
    #     self.movements += [movement]

    def __str__(self):
        return ("ini: " + str(self.wxr) + "\n"+
               "\n".join([str(m) for m in self.movements]))

    def draw(self, fps=24):
        """
        Animation of the robot doing the trajectory
        """
        wxr = self.wxr
        for move in self.movements:
            wxr,_ = animarBot(wxr, move.vc, move.t, fps)

    def getEndPosition(self):
        """
        Returns the end position resulting of performing the trajectory
        """
        wxr = self.wxr
        for move in self.movements:
            wxr,_ = simubot(move.vc, wxr, move.t)
        return wxr
