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



class LongController:
    def __init__(self, kp, ki, kd):
        
        self.pid = PIDController(kp, ki, kd)
        self.L1 = 5.34
        self.L2 = 15.2
        self.S1 = 6.84
        self.S2 = 11.17
        self.Path = 15.5
        self.x = 0
        self.v = 0
        self.v_ref = 0
        self.stop_trafic1 = False
        self.stop_trafic2 = False
        self.stop_done1 = False
        self.stop_done2 = False
        self.maxThrottle = 1
        self.time_slept = 0
        self.deadzone = 0.5
        self.switch_speed = 0.0
        self.last_stop_position = None
        self.stopsigns = None
        self.stopsigns_stoped = None
        self.stopsigns_verified = None
        self.traflights = None
        self.traflights_verified = None
        self.stay_at_red = False
        self.count = 0
    
    def update(self, v, v_ref, dt, stopsign, trafficlight, state, pos, to_print):

        pos = np.array([pos[0], pos[1]])   
        self.count += 1
        if(self.count%40==0 and to_print):
            print('Stop signs:',self.stopsigns)
            print('Traffic lights:',self.traflights)
            #print('distances to traffic lights:',car_to_traffic)
            print('state:',state)
            print('car position:',pos)
            print()

        if stopsign is not None:
            if self.stopsigns is None:
                self.stopsigns = np.array([stopsign])
                self.stopsigns_stoped = np.array([False])
                self.stopsigns_verified = np.array([False])
            else:
                if(np.linalg.norm(pos-stopsign) < 2.2):
                    #stopsignsVect = self.stopsigns - stopsign
                    distances = np.linalg.norm(self.stopsigns - stopsign,axis = 1)
                    if np.min(distances) < 1:
                        self.stopsigns_verified[np.argmin(distances)] = True

                        self.stopsigns[np.argmin(distances)] = (stopsign+self.stopsigns[np.argmin(distances)])/2
                    else:
                        self.stopsigns = np.vstack([self.stopsigns,stopsign])
                        self.stopsigns_stoped = np.append(self.stopsigns_stoped, False)
                        self.stopsigns_verified = np.append(self.stopsigns_verified, False)

        if self.stopsigns is not None and self.stopsigns_verified is not None:
            car_to_stop = np.linalg.norm(self.stopsigns[self.stopsigns_verified] - pos, axis = 1)
            if len(car_to_stop)>0:
                #print('Stop signs distance:',car_to_stop)
                min_car_to_stop = np.min(car_to_stop)
                stop_arg = np.argmin(car_to_stop)
                print('Stop signs:',self.stopsigns)
                print('Stop signs verification:',self.stopsigns_verified)
                # print('Stop signs stopped:',self.stopsigns_stoped)
                # print('stop argument',stop_arg)
                # print('Traffic lights:',self.traflights)
                # print()
                
                if (min_car_to_stop < 0.35) and (self.stopsigns_stoped[self.stopsigns_verified][stop_arg] == False):
                    v_ref = 0
                    print(self.time_slept)
                    if v < 0.1:
                        self.time_slept += dt
                        if self.time_slept > 3:
                            tmp = self.stopsigns_stoped[self.stopsigns_verified]
                            tmp[stop_arg] = True
                            self.stopsigns_stoped[self.stopsigns_verified] = tmp
                            #print(self.stopsigns_stoped)
                            #print('--------------------------------------------------------------------')
                            self.time_slept = 0
                    
        if trafficlight is not None:
            if self.traflights is None:
                self.traflights = np.array([trafficlight])
                self.traflights_verified = np.array([False])
            else:
                trafficlightsVect = self.traflights - trafficlight
                distances = np.linalg.norm(trafficlightsVect,axis = 1)
                if np.min(distances) < 1:
                    self.traflights_verified[np.argmin(distances)] = True

                    self.traflights[np.argmin(distances)] = (trafficlight+self.traflights[np.argmin(distances)])/2
                else:
                    self.traflights = np.vstack([self.traflights,trafficlight])
                    self.traflights_verified = np.append(self.traflights_verified, False)
                    
        if self.traflights is not None and self.traflights_verified is not None:
            car_to_traffic = np.linalg.norm(self.traflights[self.traflights_verified] - pos, axis = 1)          
            if len(car_to_traffic)>0:
                min_car_to_traffic = np.min(car_to_traffic)

                if min_car_to_traffic < 0.2 and state == 0:
                    self.stay_at_red = True
                if(self.stay_at_red):
                    if(state==1):
                        self.stay_at_red = False
                    else:
                        v_ref = 0
                        print('stopping due to red light')


        if self.switch_speed < v_ref:
            out = self.pid.update(v, v_ref, dt)  # Turn ON the GAZU by PID
        elif self.switch_speed >= v_ref : # Use Bang BAng to stop the car
            e = v
            if e < - self.deadzone:
                out = 1
            elif e > self.deadzone:
                out = -1
            else:
                out = 0
                
        return np.clip(out, -1, 1)

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_sum = 0
        self.prev_error = 0
    
    def update(self, v, vref, dt):
        error = vref - v
        self.error_sum += error * dt
        error_diff = (error - self.prev_error) / dt
        self.prev_error = error
        control_signal = self.kp * error + self.ki * self.error_sum + self.kd * error_diff
        return control_signal
        
