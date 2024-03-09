from pal.products.qcar import QCar, QCarCameras, QCarLidar, QCarRealSense, QCarGPS
import numpy as np
import json

from win_interface.tcp_manager import TCPManager
import yaml
import threading

import time

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
        self.cameras = cameras # Camera2D object
        self.lidar = lidar # Lidar object
        self.realSense = realSense # Camera3D object
        self.gps = gps
        
        self.isExit = False

        self.carData = dict()
        
        self.camerasData = dict()
        self.camerasDataList = list()
        self.camerasDataIterator = 0
        
        self.tcp_manager = tcp_manager
        self.tcp_manager.start_receiving()

        with open("sensorConfiguration.yaml", "r") as file:
            self.sensorConfig = yaml.safe_load(file)
            
        self.carReadoutThread = threading.Thread(target=self.readCar, args=(self, ))
        self.camerasReadoutThread = threading.Thread(target=self.readCameras, args=(self, ))
        self.carReadoutThread.start()
        self.camerasReadoutThread.start()
        self.carDataLock = threading.Lock()
        self.camerasDataLock = threading.Lock()
        
        self.timeCar = list()
        self.timeCamera = list()

    def readCar(self, obj):
        while obj.isExit is False:
            t0 = time.time()
            obj.car.read()
            obj.carDataLock.acquire()
            obj.carData["motorCurrent"] = obj.car.motorCurrent # np.zeros(2, dtype=np.float64)
            obj.carData["batteryVoltage"] = obj.car.batteryVoltage # np.zeros(2, dtype=np.float64)
            obj.carData["motorEncoder"] = obj.car.motorEncoder # np.zeros(1, dtype=np.int32)
            obj.carData["motorTach"] = obj.car.motorTach # np.zeros(1, dtype=np.float64)
            obj.carData["accelerometer"] = obj.car.accelerometer # np.zeros(3, dtype=np.float64) # x,y,z
            obj.carData["gyroscope"] = obj.car.gyroscope # np.zeros(3, dtype=np.float64) # x,y,z
            if (obj.gps is not None):
                obj.carData["isGpsAvailable"] = obj.gps.readGPS()
                obj.carData["gpsPosition"] = obj.gps.position # np.zeros((3)) # position x, position y, position z (z is 0)
                obj.carData["gpsOrientation"] = obj.gps.orientation # np.zeros((3)) # roll, pitch, yaw

                obj.carData["isLidarAvailable"] = obj.gps.readLidar()
                obj.carData["lidarDistances"] = obj.gps.distances # np.zeros((360)) # distances
                obj.carData["lidarAngles"] = obj.gps.angles # np.zeros((360)) # angles
            obj.carDataLock.release()
            obj.timeCar.append(time.time()-t0)

    def readCameras(self, obj):
        while obj.isExit is False:
            t0 = time.time()
            if (obj.cameras is not None):
                obj.cameras.readAll()
                obj.camerasDataLock.acquire()
                if (obj.cameras.csiRight is not None):
                    obj.camerasData["rightCameraData"] = obj.cameras.csiRight.imageData
                if (obj.cameras.csiBack is not None):
                    obj.camerasData["backCameraData"] = obj.cameras.csiBack.imageData
                if (obj.cameras.csiLeft is not None):
                    obj.camerasData["leftCameraData"] = obj.cameras.csiLeft.imageData
                if (obj.cameras.csiFront is not None):
                    obj.camerasData["frontCameraData"] = obj.cameras.csiFront.imageData
                obj.camerasDataLock.release()

            if (obj.realSense is not None):
                if ('rgb' in obj.sensorConfig["RealSenseMode"].lower()):
                    obj.realSense.read_RGB()
                    obj.camerasDataLock.acquire()
                    obj.camerasData["realSenseRGBData"] = obj.realSense.imageBufferRGB
                    obj.camerasDataLock.release()
                if ('depth' in obj.sensorConfig["RealSenseMode"].lower()):
                    obj.realSense.read_depth()
                    obj.camerasDataLock.acquire()
                    obj.camerasData["realSenseDepthData"] = obj.realSense.imageBufferDepthM
                    obj.camerasDataLock.release()
                if ('ir' in obj.sensorConfig["RealSenseMode"].lower()):
                    obj.realSense.read_IR()
                    obj.camerasDataLock.acquire()
                    obj.camerasData["realSenseIRDataLeft"] = obj.realSense.imageBufferIRLeft
                    obj.camerasData["realSenseIRDataRight"] = obj.realSense.imageBufferIRRight
                    obj.camerasDataLock.release()
            obj.camerasDataLock.acquire()
            obj.camerasDataList = list(obj.camerasData.keys())
            obj.camerasDataLock.release()
            obj.timeCamera.append(time.time()-t0)

    def createCarPacket(self):
        self.carDataLock.acquire()
        self.tcp_manager.send_msg(self.carData)
        self.carDataLock.release()
        
    def createCamerasPacketPart(self):
        self.camerasDataLock.acquire()
        if (len(self.camerasDataList) > 0):
            if (self.camerasDataIterator >= len(self.camerasDataList)):
                self.camerasDataIterator = 0
            self.tcp_manager.send_msg((
                self.camerasDataList[self.camerasDataIterator], # key
                self.camerasData[self.camerasDataList[self.camerasDataIterator]]) # value
                                      )
            self.camerasDataIterator += 1
        self.camerasDataLock.release()
        
def __main__():
    
    with open("tcpConfiguration.yaml", "r") as file:
            tcpConfig = yaml.safe_load(file)
    with open("sensorConfiguration.yaml", "r") as file:
            sensorConfig = yaml.safe_load(file)
    
    qcar = QCar(readMode=1, frequency=200)
    tcpManager = TCPManager(tcpConfig["ROSIP"], tcpConfig["ROSToCarPort"], tcpConfig["CarToROSPort"])
    cameras = QCarCameras(enableFront=sensorConfig["FrontUWCamEnable"],
                          enableBack=sensorConfig["BackUWCamEnable"],
                          enableLeft=sensorConfig["LeftUWCamEnable"],
                          enableRight=sensorConfig["RightUWCamEnable"])
    realrense = QCarRealSense(mode=sensorConfig["RealSenseMode"])
    gps = QCarGPS()
        
    qcarReadout = QCarReadout(
        qcar,
        tcp_manager=tcpManager,
        cameras=cameras,
        realSense=realrense,
        gps=gps
        )
    
    timeEE = list()
    tStart = time.time()
    t0 = time.time()
    
    while True:
        qcarReadout.createCarPacket()
        qcarReadout.createCamerasPacketPart()
        t = time.time()-t0
        timeEE.append(t)
        if t > 0.02:
            qcarReadout.camerasDataLock.acquire()
            print("Time exceeded:", qcarReadout.camerasDataIterator-1)
            qcarReadout.camerasDataLock.release()
        t0 = time.time()
        
        if time.time() - tStart > 10:
            break
        
    # Stop all threads safely
    qcarReadout.carDataLock.acquire()
    qcarReadout.camerasDataLock.acquire()
    qcarReadout.isExit = True
    qcarReadout.carDataLock.release()
    qcarReadout.camerasDataLock.release()
    
    qcarReadout.carReadoutThread.join()
    qcarReadout.camerasReadoutThread.join()
    qcarReadout.tcp_manager.terminate()
    
    print(max(timeEE),
          sum(timeEE)/len(timeEE),
              sum(qcarReadout.timeCar)/(len(qcarReadout.timeCar)+1),
              sum(qcarReadout.timeCamera)/(len(qcarReadout.timeCamera)+1),
              timeEE)
    print(qcarReadout.camerasDataList)
    
__main__()
        