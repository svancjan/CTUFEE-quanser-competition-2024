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

        self.carData = dict()
        self.camerasData = dict()

        self.isGpsAvailable = None
        self.gpsPosition = np.zeros((3)) # position x, position y, position z (z is 0)
        self.gpsOrientation = np.zeros((3)) # roll, pitch, yaw

        self.isLidarAvailable = None

        self.timeEE = list()
        
        self.tcp_manager = tcp_manager
        self.tcp_manager.start_receiving()

        with open("sensorConfiguration.yaml", "r") as file:
            self.sensorConfig = yaml.safe_load(file)

    def readCar(self):
        self.car.read()
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

    def readCameras(self):
        if (self.cameras is not None):
            self.cameras.readAll()
            if (self.cameras.csiRight is not None):
                self.camerasData["rightCameraData"] = self.cameras.csiRight.imageData
            if (self.cameras.csiBack is not None):
                self.camerasData["backCameraData"] = self.cameras.csiBack.imageData
            if (self.cameras.csiLeft is not None):
                self.camerasData["leftCameraData"] = self.cameras.csiLeft.imageData
            if (self.cameras.csiFront is not None):
                self.camerasData["frontCameraData"] = self.cameras.csiFront.imageData

        if (self.realSense is not None):
            if ('rgb' in self.sensorConfig["RealSenseMode"].lower()):
                self.realSense.read_RGB()
                self.camerasData["realSenseRGBData"] = self.realSense.imageBufferRGB
            if ('depth' in self.sensorConfig["RealSenseMode"].lower()):
                self.realSense.read_depth()
                self.camerasData["realSenseDepthData"] = self.realSense.imageBufferDepthM
            if ('ir' in self.sensorConfig["RealSenseMode"].lower()):
                self.realSense.read_IR()
                self.camerasData["realSenseIRDataLeft"] = self.realSense.imageBufferIRLeft
                self.camerasData["realSenseIRDataRight"] = self.realSense.imageBufferIRRight

    def create_packet(self):
        t0 = time.time()
        self.readCar()
        self.tcp_manager.send_msg(self.carData)
        self.timeEE.append(time.time()-t0)
        #self.readCameras()
        #self.tcp_manager.send_msg(self.camerasData)
        #print(sum(self.timeEE)/len(self.timeEE), end="\r")
        