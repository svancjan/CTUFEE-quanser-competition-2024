import numpy as np

from ObjectLocalization import ObjectLokalization

class LateralController:
    #LateralController(0,0,0,1)
    def __init__(self,xte_init,he_init,heLA_init,LA_ratio):
        self.xte = xte_init
        self.he = he_init
        self.heLA = heLA_init
        self.reference_delta = 0
        self.state_delta = 0
        self.C_xte = 1.4
        self.C_yaw = 4.4
        self.C_dyaw = 2
        self.LA_ratio = LA_ratio
        self.WF_state = 0

    def deltaCalc(self,errCalc,errVision,YawRate,v,dt):
        self.xte = errCalc[0] 
        self.he = errCalc[1]
        self.heLA = errCalc[2]
        delta_xte = self.XTEController(v)
        delta_dyaw = self.YawDamper(YawRate,dt)
        delta_yaw = self.YawController(delta_xte, v)
        delta_saturation = 0.6 # in degrees
        delta_ref = delta_yaw-delta_dyaw
        if(abs(delta_ref)>delta_saturation):
            delta_ref = np.sign(delta_ref)*delta_saturation
        self.reference_delta = delta_ref
        return delta_ref
    
    def XTEController(self,v):
        xte_saturation = 1 # in rad
        self.C_xte = 1.4/min(1,v)
        delta_xte = self.C_xte * self.xte
        if(abs(delta_xte)>xte_saturation):
            delta_xte = np.sign(delta_xte)*xte_saturation
        return delta_xte

    def YawController(self,delta_xte,v):
        he_err = (1-self.LA_ratio)*self.he + (self.LA_ratio)*self.heLA
        he_err = he_err*v/1.3
        delta_he = self.C_yaw*(he_err + delta_xte) 
        return delta_he
    
    def YawDamper(self,YawRate,dt):
        WF_output = self.WashOutFilter(YawRate,dt)
        delta_dyaw = self.C_dyaw*WF_output
        return delta_dyaw

    def WashOutFilter(self,yaw_rate,dt):
        WF_output = -0.02*self.WF_state + yaw_rate
        self.WF_state = self.WF_state + dt*(-0.02*self.WF_state + yaw_rate)
        return WF_output



class LongController:
    def __init__(self, kp, ki, kd):
        self.pid = PIDController(kp, ki, kd)
        self.stop_timer = 0
        self.deadzone = 0.5
        self.switch_speed = 0.01
        
        self.semaphoresLocalization = ObjectLokalization(10)
        self.stopsignsLocalization = ObjectLokalization(1.5)
        
        self.stopTimer = 0
        self.stayAtStop = False
        self.stayAtTraffic = False
    
    def update(self, v, v_ref, dt, trafficData, nnData, pos, isEnd):
        # If end, just set throttle to 0, since detection no longer needed.
        if isEnd:
            return 0
        
        if trafficData is not None:
            stopsign = trafficData[0]
            trafficlight = trafficData[1]
            state = trafficData[2]
            pos = np.array([pos[0], pos[1]])   

            self.stopsignsLocalization.localize(pos, stopsign, None)
            self.semaphoresLocalization.localize(pos, trafficlight, state == 1)
            closestTraffic = self.stopsignsLocalization.getClosest(pos)
            if closestTraffic is not None:
                min_dist, isDriveEnabled = closestTraffic
                if (min_dist < 0.40 and not isDriveEnabled):
                    self.stayAtStop = True

            
            closestTraffic = self.semaphoresLocalization.getClosest(pos)
            if closestTraffic is not None:
                min_dist, isDriveEnabled = closestTraffic
                if (min_dist < 0.3 and not isDriveEnabled):
                    self.stayAtTraffic = True
            # Has to be outside top if, since timer should continue all the time.
        if (self.stayAtStop):
            v_ref = 0
            if (v < 0.1):
                self.stopTimer += dt
                print(self.stopTimer)
                if (self.stopTimer > 3):
                    self.stayAtStop = False
                    self.stopTimer = 0
                    self.stopsignsLocalization.setClosestState(True)
        if (self.stayAtTraffic):
            if (isDriveEnabled is not None):
                self.stayAtTraffic = not isDriveEnabled
            v_ref = 0

        # localise want to receive isDriveEnabled, so when green, result will be true, otherwise (red, yellow) False
        
            # function setClosesState(True) is not needed, since localize will update state.
            # self.stopsignsLocalization.setClosestState(True)

        # if v_ref > self.switch_speed:
        #     out = self.pid.update(v, v_ref, dt)  # Turn ON the GAZU by PID
        # elif v_ref <= self.switch_speed: # Use Bang BAng to stop the car
        #     e = v
        #     if e < - self.deadzone:
        #         out = 1
        #     elif e > self.deadzone:
        #         out = -1
        #     else:
        #         out = 0
        out = self.pid.update(v, v_ref, dt)
        if(np.abs(v) < 0.2 and v_ref == 0):
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
        