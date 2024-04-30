from pal.products.qcar import QCar, QCarRealSense, QCarGPS, IS_PHYSICAL_QCAR
from hal.products.qcar import QCarEKF
import numpy as np

from tcp_manager import TCPPublisher, TCPSubscriber
import time

import psutil, os
import pickle

#MY FILES
from vehicle_model import DeltaEstimator
from path_creation import global_path
from shadow_vehicle import shadow_vehicle
from controls import LateralController, LongController
from ReferenceCalculation import reference_calc
#MY FILES

import cv2
import scipy.io

class QCarReadout():
    def __init__(self):
        self.x0 = [-1.205, -0.83, -np.pi/4]
        
        self.car = QCar(readMode=1, frequency=100)
        self.realsense = QCarRealSense(mode='RGB')
        self.gps = QCarGPS(self.x0)
        self.ekf = QCarEKF(self.x0)
        self.tcpPublisher = TCPPublisher(5556)
        self.tcpSubscriberLines = TCPSubscriber("169.254.165.100", 5555)
        self.tcpSubscriberTrafficSigns = TCPSubscriber("169.254.165.100", 5554)
        self.tcpSubscriberNeuralNetwork = TCPSubscriber("169.254.165.100", 5553)
        
        self.carData = dict()
        self.cameraData = tuple()
        
        self.throttle = 0
        self.delta = 0
        
        self.linesData = None
        self.trafficData = None
        self.neuralNetworkData = None
            
        self.Tcar = 0.0075
        self.Tcamera = 0.03
        
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
        
        print("Initialization done!")
        self.tStart = time.perf_counter()
        #MYFILES
        # initial setup
        print('Starting control setup')
        self.reference = reference_calc('QPath.npy',-1.1934, -0.8220, -0.7187)
        self.local_reference = reference_calc('QPath.npy',-1.1934, -0.8220, -0.7187)
        self.LatController = LateralController(0,0,0,1)
        self.LongController = LongController(1,0.1,0)
        self.LA_coef = 42
        self.flag_finish = False
        self.TCPenable = False
        self.TCPtime = None
        self.endTime = None
        self.p = np.array([self.x0[0], self.x0[1], self.x0[2]])
        self.deltaEstimator_ = DeltaEstimator(0.24,-0.261,-0.0202,1.0004)
        self.data_file = open("Local.txt", "a")
        #MYFILES'
        
    def calcEkf(self, yawRate, timeDiff):
        isNewGps = self.gps.readGPS()
        
        self.deltaEstimator_.calcDelta(self.delta, timeDiff)
        
        if isNewGps:
            self.p[0] = self.gps.position[0]
            self.p[1] = self.gps.position[1]
            self.p[2] = self.gps.orientation[2]

            y_gps = np.array([
                self.gps.position[0],
                self.gps.position[1],
                self.gps.orientation[2]
            ])
            self.ekf.update(
                [0.96*self.car.motorTach, self.deltaEstimator_.getActualDelta()],
                timeDiff,
                y_gps,
                yawRate,
            )
        else:
            self.ekf.update(
                [0.96*self.car.motorTach, self.deltaEstimator_.getActualDelta()],
                timeDiff,
                None,
                yawRate,
            )
        
    def readCar(self,timeDiff):
        self.car.read()
        self.calcEkf(self.car.gyroscope[2], timeDiff)
        self.carData["kalmanFilter"] = np.array(
            [self.ekf.x_hat[0,0], self.ekf.x_hat[1,0], self.ekf.x_hat[2,0]])# x, y, theta
        
    def readRGBCamera(self):
        self.realsense.read_RGB()
        if (self.realsense.imageBufferRGB is not None):
            if(self.TCPtime is not None and not self.flag_finish):
                self.cameraData = (self.realsense.imageBufferRGB, self.p, time.perf_counter()-self.TCPtime)
            elif(self.TCPtime is not None):
                self.cameraData = (self.realsense.imageBufferRGB, self.p, self.endTime)
            else:
                self.cameraData = (self.realsense.imageBufferRGB, self.p, 0)
            self.tcpPublisher.send_msg(pickle.dumps(self.cameraData, protocol=pickle.DEFAULT_PROTOCOL))
            
    def receiveLines(self):
        msg = self.tcpSubscriberLines.receive_msg()
        if (msg is not None):
            self.TCPenable = True
            if (self.TCPtime is None):
                self.TCPtime = time.perf_counter()
            self.linesData = pickle.loads(msg)
            if self.linesData is not None:
                if len(self.linesData)>20:
                    self.local_reference.update_path(self.linesData)
                 
        
    def receiveTrafficSigns(self):
        msg = self.tcpSubscriberTrafficSigns.receive_msg()
        if (msg is not None):
            self.TCPenable = True
            if (self.TCPtime is None):
                self.TCPtime = time.perf_counter()
            self.trafficData = pickle.loads(msg)
            
    def receiveNeuralNetworkData(self):
        msg = self.tcpSubscriberNeuralNetwork.receive_msg()
        if (msg is not None):
            self.TCPenable = True
            if (self.TCPtime is None):
                self.TCPtime = time.perf_counter()
            self.neuralNetworkData = pickle.loads(msg)
            
    def pushControl(self,timeDiff) -> bool:
        # Wait for TCP
        if(self.TCPenable):
            # actual velocity was experimentaly estimated as follows
            v = 0.96*self.car.motorTach

            # Calculate model vehicle position
            self.p[0] += timeDiff * v * np.cos(self.p[2])
            self.p[1] += timeDiff * v * np.sin(self.p[2])
            self.p[2] += timeDiff * v *  np.tan(self.deltaEstimator_.getActualDelta())/0.2

            # Calculate cross-track and yaw error from reference
            ref = self.reference.update(self.p[0],self.p[1],self.p[2],v,self.LA_coef)
            ref_loc = self.local_reference.update(self.p[0],self.p[1],self.p[2],v,0)
                        
            # wait after TCP is enables, for 1 second as initial startup
            if(time.perf_counter()-self.TCPtime>1):
                self.data_file.write(str(ref[1]) + "," + str(ref_loc[1]) + "," + str(ref[2]) + "," + str(ref_loc[2]) + "," + str((time.perf_counter()-self.TCPtime)-1) + ",")
                self.data_file.write(str(self.reference.sv.shadow_vehicle.x) + "," + str(self.reference.sv.shadow_vehicle.y) + "," + str(self.reference.sv.shadow_vehicle.psi) + ",")
                self.data_file.write(str(self.local_reference.sv.shadow_vehicle.x) + "," + str(self.local_reference.sv.shadow_vehicle.y) + "," + str(self.local_reference.sv.shadow_vehicle.psi) + "\n")
            
                #v ref calculation
                v_ref = ref[0]
                
                if(time.perf_counter()-self.TCPtime<2):
                    v_ref = 1.5 #for debugging
                
                # Calculate yaw rate from the estimated actual steering angle
                if(abs(self.deltaEstimator_.getActualDelta())>0.01):
                    radius = 0.2/np.tan(self.deltaEstimator_.getActualDelta())
                    yaw_rateCalc = v/radius  
                else:
                    yaw_rateCalc = 0
                    
                # Calculate steering angle
                self.delta = self.LatController.deltaCalc(ref[1:],ref_loc[1:],yaw_rateCalc,v,timeDiff)

                # Check if car is in the finish line
                if(np.linalg.norm(self.p[:2]-np.array([-1.984125,0.47703]))<0.5):
                    v_ref = 0

                if(np.linalg.norm(self.p[:2]-np.array([-1.984125,0.47703]))<0.5 and v<0.3):
                    self.flag_finish = True
                    print('total time:',time.perf_counter()-self.TCPtime)
                    self.endTime = time.perf_counter()-self.TCPtime
                    self.throttle = 0
                    self.delta = 0
                    
                
                # Calculate throttle    
                else:  
                    self.throttle = self.LongController.update(v, v_ref, timeDiff, self.trafficData, self.neuralNetworkData, self.p, self.flag_finish)
                
        self.car.write(throttle=self.throttle, steering=self.delta, LEDs=None)
        return self.flag_finish

def __main__():
    p = psutil.Process(os.getpid())
    p.nice(psutil.REALTIME_PRIORITY_CLASS)

    qcarReadout = QCarReadout()
    
    t0 = time.perf_counter()
    t = 0
    tp = None
    isEnd = False
    readCameraSchedule = time.perf_counter() + qcarReadout.Tcamera
    while(isEnd == False):
        tp = t
        t = time.perf_counter() - t0
        dt = t-tp
        qcarReadout.readCar(dt)
        if(time.perf_counter() > readCameraSchedule):
            readCameraSchedule = time.perf_counter() + qcarReadout.Tcamera
            qcarReadout.readRGBCamera()
        qcarReadout.receiveLines()
        qcarReadout.receiveTrafficSigns()
        qcarReadout.receiveNeuralNetworkData()
        isEnd = qcarReadout.pushControl(dt)
    
__main__()
