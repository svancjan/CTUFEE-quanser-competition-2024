
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, Float32
from Deserializer import deserialize_message, serialize_message
import time
import sys
import numpy as np


class SpeedFeedForwardController:
    def __init__(self, k=1):
        self.L1 = 5.34
        self.L2 = 15.2
        self.S1 = 6.84
        self.S2 = 11.17
        self.Path = 15.5
        self.x = 0
        self.v = 0
        self.v_ref = 0
        self.stop_done1 = False
        self.stop_done2 = False
        self.u_last = 0
        self.maxThrottle = 1
        self.time_slept = 0
        #self.stop_distance = 0.51 #without controll
        self.stop_distance = 0.1 #with control


    def update(self, v, dt):
        emergency_stop = False
        u = self.u_last
        a = -1/0.24 * v + 9.4/0.24 * u
        #v = a*dt
        #self.v += v
        self.x += v*dt
        safety = 0.1
        
        # First run to Stop 1
        if self.x < (self.S1 - self.stop_distance):
            self.v_ref = 4
        else: 
            if self.stop_done1 == False:
                self.v_ref = 0
                # wait for 3 seconds
                if(self.time_slept < 3):
                    self.time_slept += dt
                else:
                    self.v_ref = 4
                    self.stop_done1 = True
                    self.time_slept = 0
                
            
            if (self.x < self.S2 - self.stop_distance) and (self.stop_done1 == True):
                self.v_ref = 4
            elif(self.stop_done1 == True):
                if self.stop_done2 == False:
                    self.v_ref = 0
                    print("waiting at stop 2", self.time_slept)
                    # wait 3 seconds
                    if(self.time_slept < 3) and (self.stop_done2 == False):
                        self.time_slept += dt
                    else:  
                        self.v_ref = 4
                        self.stop_done2 = True
                        self.time_slept = 0

                if (self.x < self.Path) and (self.stop_done2 == True):
                    self.v_ref = 4
                if (self.x > self.Path) and (self.stop_done2 == True):
                    self.v_ref = 0
                    print("I am at the finish line")

        #if emergency_stop:
        #    self.v_ref = 0

        deadzone = 0.25

        print("v_ref: ", self.v_ref, self.x)
        if 1 < self.v_ref:
            out = 1.0  # Turn ON the GAZU
        elif 1 > self.v_ref :
            out = 0.0  # Turn OFF the GAZU
            
            e = v
            if e < -deadzone:
                out = 0.0
                out = 1.0
            elif e > deadzone:
                out = 0.0
                out = -1.0
            else:
                out = 0.0
        
        else:
            out = 0.0
        
            
        self.u_last = out
        return out



class logitudunalcontrol(Node):
    def __init__(self):
        super().__init__('my_subscriber')

        self.controler = SpeedFeedForwardController() 
        self.t0 = time.perf_counter()
        self.startDelay = 0.1
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.v = None
        self.u = 0
        self.t = None
        self.controlPublisher = self.create_publisher(ByteMultiArray, 'CarControl', qos_profile = qos_profile)
        self.controlTimer = self.create_timer(0.025, self.publishData)

        # Create a subscriber to the desired topic
        self.Car_subs = self.create_subscription(
            ByteMultiArray,
            'CarData',
            self.processCarData,
            qos_profile = qos_profile,
            raw=True
        )
        
        self.Vison_subs = self.create_subscription(
            ByteMultiArray,
            'VisionData',
            self.processVisionData,
            qos_profile = qos_profile,
            raw=True
        )


        self.controlPublisher = self.create_publisher(Float32, 'CarControl_Longitudal', qos_profile = qos_profile)

    def processCarData(self, message):
        message = deserialize_message(message)
        #print("Received msg type: ", type(message))
        self.v = message["motorTach"]
         
        

    def processVisionData(self, message):
        return #print("Received msg type: ", type(deserialize_message(message)))

    
    def publishData(self):
        if self.v is not None:
            # get dt
            tp = self.t
            self.t = time.perf_counter() - self.t0
            if self.t is not None and tp is not None:
                dt = self.t-tp
                if self.t < self.startDelay:
                    self.u = 0
                else:
                    self.u = self.controler.update(self.v, dt)
                    #print(self.u, type(self.u))
                    outMsg = Float32()
                    outMsg.data = self.u
                    self.controlPublisher.publish(outMsg)

    

def __main__():
    rclpy.init()
    node = logitudunalcontrol()
    rclpy.spin(node)

__main__()

