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
        self.u_last = 0
        self.maxThrottle = 1
        self.time_slept = 0
        #self.stop_distance = 0.51 #without controll
        self.stop_distance = 0.1 #with control
        self.deadzone = 0.15
        self.switch_speed = 0.0
    
    def update(self, v, v_ref, dt, stopsign, trafficlight, distance, state):
        #red is 0
        if (trafficlight is not None) and (state = 0):

            if (distance < 1.9):
                v_ref = 0

        if (stopsign is not None) and (distance < 0.4):
            if self.stop_done1 is False:
                v_ref = 0
                if v == 0:
                    self.time_slept += dt
                    if self.time_slept > 3:
                        self.stop_done1 = True
                        self.time_slept = 0
            elif self.stop_done2 is False:
                v_ref = 0
                if v == 0:
                    self.time_slept += dt
                    if self.time_slept > 3:
                        self.stop_done2 = True
                        self.time_slept = 0
        
        

        if self.switch_speed < v_ref:
            out = self.pid.update(v, v_ref, dt)  # Turn ON the GAZU by PID
        elif self.switch_speed >= v_ref : # Use Bang BAng to stop the car
            e = v
            if e < - self.deadzone:
                out = 0
                out = 1
            elif e > self.deadzone:
                out = 0
                out = -1
            else:
                out = 0    
        else:
            out = 0
        
            
        self.u_last = out
        #dont need numpy
        if out > 1:
            out = 1
        elif out < -1:
            out = -1

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
