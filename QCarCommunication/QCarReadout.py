from pal.products.qcar import QCar, QCarCameras, QCarLidar, QCarRealSense, QCarGPS, IS_PHYSICAL_QCAR
#import numpy as np

from win_interface.tcp_manager import TCPManager
import yaml

import time
from sched import scheduler

import psutil, os # Namrdam tam "RT" a pofrci to huhuhuhahaha


class QCarReadout():
    def __init__(
            self,
            car : QCar,
            tcp_manager : TCPManager,
            cameras : QCarCameras = None,
            lidar : QCarLidar = None,
            realSense : QCarRealSense = None,
            gps : QCarGPS = None,
            ):
        self.car = car
        self.cameras = cameras # Camera2D obj
        self.lidar = lidar # Lidar obj
        self.realSense = realSense # Camera3D obj
        self.gps = gps
        
        self.isExit = False

        self.carData = dict()
        
        self.cameraData = tuple()
        
        self.tcp_manager = tcp_manager

        with open("sensorConfiguration.yaml", "r") as file:
            self.sensorConfig = yaml.safe_load(file)
            
        self.scheduler = scheduler(time.perf_counter, time.sleep)
        self.Tcar = 0.005
        self.Tcamera = 0.020
        self.Tcontrol = 0.005
        
        self.timeCar = list()
        self.timeCamera = list()
        self.timeControl = list()
        self.timeFeedback = list()
        
        self.carT0 = None
        self.camT0 = None
        self.conT0 = None
        
        self.camSchedHandle = self.scheduler.enterabs(time.perf_counter() + self.Tcamera, 3, self.readRGBCamera)
        self.carSchedHandle = self.scheduler.enterabs(time.perf_counter() + self.Tcar, 2, self.readCar)
        self.controlSchedHandle = self.scheduler.enterabs(time.perf_counter() + self.Tcontrol, 1, self.pushControl)
        
        self.carCounter = 0
        self.feedbackCounter = 0

    def terminate(self):
        self.scheduler.cancel(self.camSchedHandle)
        self.scheduler.cancel(self.carSchedHandle)
        self.scheduler.cancel(self.controlSchedHandle)
        self.tcp_manager.terminate()

    def readCar(self):
        self.carSchedHandle = self.scheduler.enterabs(time.perf_counter() + self.Tcar, 2, self.readCar)
        self.car.read()
        self.carData["timeStamp"] = time.perf_counter()
        self.carData["motorCurrent"] = self.car.motorCurrent # np.zeros(2, dtype=np.float64)
        self.carData["batteryVoltage"] = self.car.batteryVoltage # np.zeros(2, dtype=np.float64)
        self.carData["motorEncoder"] = self.car.motorEncoder # np.zeros(1, dtype=np.int32)
        self.carData["motorTach"] = self.car.motorTach # np.zeros(1, dtype=np.float64)
        self.carData["accelerometer"] = self.car.accelerometer # np.zeros(3, dtype=np.float64) # x,y,z
        self.carData["gyroscope"] = self.car.gyroscope # np.zeros(3, dtype=np.float64) # x,y,z
        if (self.gps is not None):
            self.carData["isGpsAvailable"] = self.gps.readGPS()
            self.carData["gpsPosition"] = self.gps.position # np.zeros((3)) # position x, position y, position z (z is 0)
            self.carData["gpsOrientation"] = self.gps.orientation # np.zeros((3)) # roll, pitch, yaw
            
            self.carData["isLidarAvailable"] = self.gps.readLidar()
            self.carData["lidarDistances"] = self.gps.distances # np.zeros((360)) # distances
            self.carData["lidarAngles"] = self.gps.angles # np.zeros((360)) # angles
        self.tcp_manager.send_msg(self.carData)
        self.carCounter += 1
        if self.carT0 is not None:
            self.timeCar.append(time.perf_counter()-self.carT0)
        self.carT0 = time.perf_counter()

    def readRGBCamera(self):
        t0 = time.perf_counter()
        self.camSchedHandle = self.scheduler.enterabs(t0 + self.Tcamera, 3, self.readRGBCamera)
        self.cameras.readAll()
        if (self.cameras.csiFront is not None):
            self.cameraData = (self.cameras.csiFront.imageData, t0)
            self.tcp_manager.send_msg(self.cameraData)
            if self.camT0 is not None:
                self.timeCamera.append(time.perf_counter()-self.camT0)
            self.camT0 = time.perf_counter()

    def pushControl(self):
        self.controlSchedHandle = self.scheduler.enterabs(time.perf_counter() + self.Tcontrol, 1, self.pushControl)
        message = self.tcp_manager.receive_msg(0) # u, delta
        if (message is not None):
            self.feedbackCounter += 1
            self.car.write(throttle=message[0], steering=message[1], LEDs=None)
            self.timeFeedback.append(time.perf_counter()-message[2])
            if self.conT0 is not None:
                self.timeControl.append(time.perf_counter()-self.conT0)
            self.conT0 = time.perf_counter()

def __main__():
    
    with open("tcpConfiguration.yaml", "r") as file:
            tcpConfig = yaml.safe_load(file)
    with open("sensorConfiguration.yaml", "r") as file:
            sensorConfig = yaml.safe_load(file)
    
    qcar = QCar(readMode=1, frequency=100)
    tcpManager = TCPManager(tcpConfig["ROSIP"], tcpConfig["ROSToCarPort"], tcpConfig["CarToROSPort"])
    cameras = QCarCameras(enableFront=sensorConfig["FrontUWCamEnable"],
                          enableBack=sensorConfig["BackUWCamEnable"],
                          enableLeft=sensorConfig["LeftUWCamEnable"],
                          enableRight=sensorConfig["RightUWCamEnable"])
    realrense = QCarRealSense(mode=sensorConfig["RealSenseMode"])
    
    del sensorConfig, tcpConfig
    import gc
    gc.collect()
    
    gps = QCarGPS()
    
    qcarReadout = QCarReadout(
        qcar,
        tcp_manager=tcpManager,
        cameras=cameras,
        realSense=realrense,
        gps=gps
        )
    
    qcarReadout.scheduler.enterabs(time.perf_counter() + 10, 1, qcarReadout.terminate)
    qcarReadout.scheduler.run()
    
    print("{:<8} {:<8} {:<8} ".format('name','max','mean'))
    print("{:<8} {:<8} {:<8} ".format(
        "car","{:.1f}".format(max(qcarReadout.timeCar)*1000), 
          "{:.1f}".format(sum(qcarReadout.timeCar)/(len(qcarReadout.timeCar))*1000)))
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


p = psutil.Process(os.getpid())
p.nice(psutil.HIGH_PRIORITY_CLASS) # NAZDAAAAR

# Ensure that timer resolution is 1 ms (can be up to 16)
#from ctypes import c_int, windll, byref
#originalRes = c_int()
#windll.ntdll.NtSetTimerResolution(10000, True, byref(originalRes))
    
#__main__()
import pstats, cProfile
cProfile.run('__main__()','restats')
stat = pstats.Stats('restats')
stat.sort_stats('cumulative').print_stats(20)

# Slow down system timer again
#windll.ntdll.NtSetTimerResolution(156250, True, byref(originalRes))
        