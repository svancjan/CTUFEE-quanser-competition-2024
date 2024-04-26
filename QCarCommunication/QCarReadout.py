from pal.products.qcar import QCar, QCarRealSense, QCarGPS, IS_PHYSICAL_QCAR
from hal.products.qcar import QCarEKF
import numpy as np

from win_interface.tcp_manager import TCPPublisher, TCPSubscriber
import time

import psutil, os
import pickle

#MY FILES
from vehicle_model import vehicle_position
from path_creation import global_path
from shadow_vehicle import shadow_vehicle
from controls import LateralController, LongController
from ReferenceCalculation import reference_calc
#MY FILES

import cv2

class QCarReadout():
    def __init__(self):
        self.x0 = [-1.19, -0.82, -0.7187]
        
        self.car = QCar(readMode=1, frequency=100)
        self.realsense = QCarRealSense(mode='RGB')
        self.gps = QCarGPS(self.x0)
        self.ekf = QCarEKF(self.x0)
        self.tcpPublisher = TCPPublisher(5556)
        self.tcpSubscriberLines = TCPSubscriber("169.254.165.100", 5555)
        self.tcpSubscriberTrafficSigns = TCPSubscriber("169.254.165.100", 5554)
        
        self.carData = dict()
        self.cameraData = tuple()
        
        self.gyroData = None
        self.throttle = 0
        self.delta = 0
        
        self.linesData = None
        self.trafficData = None
            
        self.Tcar = 0.0075
        self.Tcamera = 0.04
        
        self.carCounter = 0
        self.feedbackCounter = 0
        
        # DEBUG begin
        self.timeCar = list()
        self.timeCamera = list()
        self.timeControl = list()
        self.timeFeedback = list()
        self.timeEkf = list()
        
        self.carT0 = None
        self.camT0 = None
        self.conT0 = None
        # DEBUG end
        
        self.startDelay = 1.5 # s 
        #self.estimatedDelta = 0
        
        print("Initialization done!")
        self.tStart = time.perf_counter()
        #MYFILES
        # initial setup
        print('Starting control setup')
        self.reference = reference_calc('QPath.npy',-1.1934, -0.8220, -0.7187)
        self.LatController = LateralController(0,0,0,1)
        self.LongController = LongController(1,0.1,0)
        self.flag_init = True
        self.flag_HA = True
        self.flag_finish = True
        self.TCPenable = False
        self.TCPtime = None
        self.StopSignEstimate = None
        self.StopSignTime = None
        self.endTime = None
        self.p = np.array([self.x0[0], self.x0[1], self.x0[2]]) + np.array([np.cos(self.x0[2]), np.sin(self.x0[2]), 0]) * 0.2
        #MYFILES'
        
    def calcEkf(self, yawRate, timeDiff):
        isNewGps = self.gps.readGPS()
        
        if(time.perf_counter() - self.tStart > 1.2):
            self.estimatedDelta = self.estimatedDelta + (timeDiff)*(-6.25*self.estimatedDelta + 6.25*self.delta)
        else:
            self.estimatedDelta = self.delta
        
        if isNewGps:
            y_gps = np.array([
                self.gps.position[0],
                self.gps.position[1],
                self.gps.orientation[2]
            ])
            self.ekf.update(
                [self.car.motorTach, self.estimatedDelta],
                timeDiff,
                y_gps,
                yawRate,
            )
        else:
            self.ekf.update(
                [self.car.motorTach, self.estimatedDelta],
                timeDiff,
                None,
                yawRate,
            )
        
    def readCar(self,timeDiff):
        self.car.read()
        self.calcEkf(self.car.gyroscope[2], timeDiff)
        #self.carData["kalmanFilter"] = np.array(
        #    [self.ekf.x_hat[0,0], self.ekf.x_hat[1,0], self.ekf.x_hat[2,0]])# x, y, theta
        
        # DEBUG begin
        self.carCounter += 1
        if self.carT0 is not None:
            self.timeCar.append(time.perf_counter()-self.carT0)
        self.carT0 = time.perf_counter()
        # DEBUG end
        
    def readRGBCamera(self):
        t0 = time.perf_counter()
        self.realsense.read_RGB()
        if (self.realsense.imageBufferRGB is not None):
            if(self.TCPtime is not None and self.flag_finish):
                self.cameraData = (self.realsense.imageBufferRGB, self.p, time.perf_counter()-self.TCPtime)
            elif(self.TCPtime is not None):
                self.cameraData = (self.realsense.imageBufferRGB, self.p, self.endTime)
            else:
                self.cameraData = (self.realsense.imageBufferRGB, self.p, 0)
            self.tcpPublisher.send_msg(pickle.dumps(self.cameraData, protocol=pickle.DEFAULT_PROTOCOL))
            # DEBUG begin
            #cv2.imwrite("drive\\" + str(round((time.perf_counter()-self.tStart)*1000)) + ".jpg", self.cameraData[0])
            #cv2.imshow('RGB', self.cameraData[0])
            #cv2.waitKey(1)
            if self.camT0 is not None:
                self.timeCamera.append(time.perf_counter()-self.camT0)
            self.camT0 = time.perf_counter()
            # DEBUG end
            
    def receiveLines(self):
        msg = self.tcpSubscriberLines.receive_msg()
        if (msg is not None):
            self.TCPenable = True
            self.linesData = pickle.loads(msg)
        
    def receiveTrafficSigns(self):
        msg = self.tcpSubscriberTrafficSigns.receive_msg()
        if (msg is not None):
            self.trafficData = pickle.loads(msg)
            
    def pushControl(self,timeDiff):
        # Wait for TCP
        if(self.TCPenable):
            self.p = np.array([self.ekf.x_hat[0,0], self.ekf.x_hat[1,0], self.ekf.x_hat[2,0]]) + np.array([np.cos(self.ekf.x_hat[2,0]), np.sin(self.ekf.x_hat[2,0]), 0]) * 0.2

            v = self.car.motorTach
            #print('     velocity:',v)
            #print('-----------velocityy',v)
            LA_coef = int(18*min(1,v))
            ref = self.reference.update(self.p[0],self.p[1],self.ekf.x_hat[2,0],v,LA_coef)
            
            # controller update
            if(self.flag_HA): #print only
                self.TCPtime = time.perf_counter()
                print('HA starting')
                self.flag_HA = False
            if(time.perf_counter()-self.TCPtime>1):
                
                #v ref calculation
                v_ref = ref[0]
                if(v_ref>3):
                    v_ref = 2.8 #for debugging
                
                self.delta = self.LatController.deltaCalc(ref[1],ref[2],ref[3],self.car.gyroscope[2],v)
                                
                # Check if car is in the finish line
                if(np.linalg.norm(self.p[:2]-np.array([-1.984125,0.47703]))<0.5 and v<0.1):
                    if(self.flag_finish):
                        print('finish')
                        self.flag_finish = False
                        print('total time:',time.perf_counter()-self.TCPtime)
                        self.endTime = time.perf_counter()-self.TCPtime
                        exit()
                        
            # Initial control
            #else:
            #    if(self.flag_init):
            #        print('controls init')
            #        self.flag_init = False
            #    v_ref = 0.7
            #    self.delta = 0.25 #update(self, v, v_ref, dt, stopsign, trafficlight, state):
                #print('position:',self.p)
                self.throttle = self.LongController.update(v, v_ref, timeDiff, self.trafficData[0], self.trafficData[1] , self.trafficData[2], self.p, self.flag_finish)
        else:
            self.delta = 0
            self.throttle = 0
        self.car.write(throttle=self.throttle, steering=self.delta, LEDs=None)

def __main__():
    p = psutil.Process(os.getpid())
    p.nice(psutil.REALTIME_PRIORITY_CLASS)
    
    # Ensure that timer resolution is 1 ms (can be up to 16)
    #from ctypes import c_int, windll, byref
    #originalRes = c_int()
    #windll.ntdll.NtSetTimerResolution(10000, True, byref(originalRes))

    qcarReadout = QCarReadout()
    
    t0 = time.perf_counter()
    t = 0
    tp = None
    readCarSchedule = time.perf_counter() + qcarReadout.Tcar
    readCameraSchedule = time.perf_counter() + qcarReadout.Tcamera
    while(time.perf_counter()-t0 < 60):
        tp = t
        t = time.perf_counter() - t0
        dt = t-tp
        #print('dt:',dt) 
        if(time.perf_counter() > readCarSchedule):
            readCarSchedule = time.perf_counter() + qcarReadout.Tcar
            qcarReadout.readCar(dt)
        if(time.perf_counter() > readCameraSchedule):
            readCameraSchedule = time.perf_counter() + qcarReadout.Tcamera
            qcarReadout.readRGBCamera()
        qcarReadout.receiveLines()
        qcarReadout.receiveTrafficSigns()
        qcarReadout.pushControl(dt)
    print('Simulation done!')
    
    # Slow down system timer again
    #windll.ntdll.NtSetTimerResolution(156250, True, byref(originalRes))
    
    # DEBUG begin
    print("{:<8} {:<8} {:<8} ".format('name','max','mean'))
    print("{:<8} {:<8} {:<8} ".format(
        "car","{:.1f}".format(max(qcarReadout.timeCar)*1000), 
          "{:.1f}".format(sum(qcarReadout.timeCar)/(len(qcarReadout.timeCar))*1000)))
    print("{:<8} {:<8} {:<8} ".format(
        "ekf","{:.1f}".format(max(qcarReadout.timeEkf)*1000), 
          "{:.1f}".format(sum(qcarReadout.timeEkf)/(len(qcarReadout.timeEkf))*1000)))
    print("{:<8} {:<8} {:<8} ".format(
        "camera","{:.1f}".format(max(qcarReadout.timeCamera)*1000), 
          "{:.1f}".format(sum(qcarReadout.timeCamera)/(len(qcarReadout.timeCamera))*1000)))
    print("{:<8} {:<8} {:<8} ".format(
        "control","{:.1f}".format(max(qcarReadout.timeControl)*1000),
          "{:.1f}".format(sum(qcarReadout.timeControl)/(len(qcarReadout.timeControl))*1000)))
    print("{:<8} {:<8} {:<8} ".format(
        "feedback","{:.1f}".format(max(qcarReadout.timeFeedback)*1000), 
          "{:.1f}".format(sum(qcarReadout.timeFeedback)/(len(qcarReadout.timeFeedback))*1000)))
    print("counters:", qcarReadout.carCounter, qcarReadout.feedbackCounter)
    # DEBUG end
    
__main__()


# DEBUG begin
#import pstats, cProfile
#cProfile.run('__main__()','restats')
#stat = pstats.Stats('restats')
#stat.sort_stats('cumulative').print_stats(20)
# DEBUG end
