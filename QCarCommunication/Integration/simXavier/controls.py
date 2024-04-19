import numpy as np
from vehicle_model import vehicle_position, KinematicVehicleModel
from path_creation import global_path
import matplotlib.pyplot as plt
from shadow_vehicle import shadow_vehicle
import csv


class LateralController:
    def __init__(self,xte_init,he_init,heLA_init,LA_ratio):
        self.xte = xte_init
        self.he = he_init
        self.heLA = heLA_init
        self.reference_delta = 0
        self.state_delta = 0
        self.C_xte = 1.3
        self.C_yaw = 2
        self.C_dyaw = 0.7
        self.LA_ratio = LA_ratio

    def deltaCalc(self,xte,he,heLA,YawRate,v):
        self.xte = xte
        self.he = he
        self.heLA = heLA
        delta_xte = self.XTEController(v)
        delta_dyaw = self.YawDamper(YawRate)
        delta_yaw = self.YawController(delta_xte)
        delta_saturation = 0.6 # in degrees
        delta_ref = delta_yaw-delta_dyaw
        if(abs(delta_ref)>delta_saturation):
            delta_ref = np.sign(delta_ref)*delta_saturation
        self.reference_delta = delta_ref
        return delta_ref
    
    def XTEController(self,v):
        xte_saturation = 0.5 # in rad
        self.C_xte = 1.3/min(1,max(v,0.1))
        delta_xte = self.C_xte * self.xte
        if(abs(delta_xte)>xte_saturation):
            delta_xte = np.sign(delta_xte)*xte_saturation
        return delta_xte

    def YawController(self,delta_xte):
        he_err = (1-self.LA_ratio)*self.he + (self.LA_ratio)*self.heLA
        delta_he = self.C_yaw*(he_err + delta_xte) 
        return delta_he
    
    def YawDamper(self,YawRate):
        delta_dyaw = self.C_dyaw*YawRate
        return delta_dyaw



class LongitudinalController:
    def __init__(self, kp=0, ki=0):
        self.maxThrottle = 0.3

        self.kp = kp
        self.ki = ki

        self.ei = 0


    # ==============  SECTION A -  Speed Control  ====================
    def update(self, v, v_ref, dt):
        
        e = v_ref - v
        self.ei += dt*e

        return np.clip(
            self.kp*e + self.ki*self.ei,
            -self.maxThrottle,
            self.maxThrottle
        )
        
