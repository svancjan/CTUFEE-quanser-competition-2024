import os
import signal
import numpy as np
from threading import Thread
import time
import cv2
import pyqtgraph as pg

from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.utilities.scope import MultiScope
from pal.utilities.math import wrap_to_pi
from hal.products.qcar import QCarEKF
from hal.products.mats import SDCSRoadMap
import pal.resources.images as images

import numpy as np
from vehicle_model import vehicle_position
from path_creation import global_path
from shadow_vehicle import shadow_vehicle

#MYFILES
class my_quanser_car():
    def __init__(self, x0, y0, psi0):
        self.position = vehicle_position(x0,y0,psi0)
#MYFILES


class reference_calc:
    def __init__(self,trajectory_filename,x0,y0,psi0):
        self.traj = global_path(trajectory_filename)
        self.traj.get_global_path(151,False,'linear')
        #self.my_car = my_quanser_car(-1.1934, -0.8220, -0.7187)
        self.my_car = my_quanser_car(x0,y0,psi0)
        self.sv = shadow_vehicle(self.my_car,self.traj)
        self.sv.get_shadow_position(self.my_car)

    def update(self,x,y,psi,v,LA_coef):
        self.my_car.position.x = x
        self.my_car.position.y = y
        self.my_car.position.psi = psi
        self.sv.get_shadow_position(self.my_car)
        self.sv.crossTrackCalc(self.my_car)
        self.sv.headingErrCalc(self.my_car)
        self.sv.headingErrCalcLA(self.my_car,v,LA_coef)
        xte = self.sv.xte
        he = self.sv.HeadingErr
        heLA = self.sv.HeadingErrLA
        v_ref = self.sv.v_ref
        return np.array([v_ref, xte, he, heLA])
                