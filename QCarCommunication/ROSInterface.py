import threading
import numpy as np
from std_msgs.msg import  ByteMultiArray, MultiArrayLayout, MultiArrayDimension
from win_interface.tcp_manager import TCPManager
import yaml
import time
import pickle
import sys

import rclpy
from rclpy.node import Node

class QCarDataPublisher(Node):
    def __init__(self) -> None:
        super().__init__('qcarDataPublisher')
        self.shutDownRequired = False
        
        with open("tcpConfiguration.yaml", "r") as file:
            config = yaml.safe_load(file)

        # Create TCPManager instance
        self.tcp_manager = TCPManager(config["QCarIP"], config["CarToROSPort"], config["ROSToCarPort"])
        self.carData = None
        self.camerasData = dict()
        
        self.carDataLock = threading.Lock()
        self.camerasDataLock = threading.Lock()
        
        self.carDataPublisher = self.create_publisher(ByteMultiArray, 'CarData', 1)
        self.rgbCameraPublisher = self.create_publisher(ByteMultiArray, 'CameraRGB', 1)
        self.depthCameraPublisher = self.create_publisher(ByteMultiArray, 'CameraDepth', 1)
        self.frontCameraPublisher = self.create_publisher(ByteMultiArray, 'CameraFront', 1)
        self.cameraTimer = self.create_timer(0.05, self.sendCameraData)
        self.carTimer = self.create_timer(0.005, self.sendCarData)
        
        self.camerasData["realSenseRGBData"] = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        self.carData = dict()
        self.carData["batteryVoltage"] = np.random.uniform(10.0, 14.0)
        self.carData["motorSpeed"] = np.random.randint(0, 100)
        self.carData["temperature"] = np.random.uniform(20.0, 40.0)
        self.carData["position"] = np.random.uniform(0.0, 100.0, size=(3,))
        self.carData["orientation"] = np.random.uniform(0.0, 360.0, size=(3,))
        self.carData["lidarData"] = np.random.uniform(0.0, 100.0, size=(360,))
    
    def cyclicReceiveTCP(self):        
        while True:
            msg = self.tcp_manager.receive_msg()
            if (msg is not None):
                if (type(msg) == dict):
                    self.carDataLock.acquire()
                    self.carData = msg
                    self.carDataLock.release()
                elif (type(msg) == tuple):
                    self.camerasDataLock.acquire()
                    self.camerasData[msg[0]] = msg[1]
                    self.camerasDataLock.release()
            continue
        
    def sendCameraData(self):
        t0 = time.time()
        if rclpy.ok() and "realSenseRGBData" in self.camerasData.keys():
            self.camerasDataLock.acquire()
            array_msg = pickle.dumps(self.camerasData["realSenseRGBData"],protocol=pickle.DEFAULT_PROTOCOL)
            self.camerasDataLock.release()
            self.rgbCameraPublisher.publish(array_msg)
            print("Publishing RS RGB data to ROS took: ", (time.time()-t0)*1000," ms")
            
    def sendCarData(self):
        t0 = time.time()
        if rclpy.ok() and self.carData is not None:
            self.carDataLock.acquire()
            array_msg = pickle.dumps(self.carData,protocol=pickle.DEFAULT_PROTOCOL)
            self.carDataLock.release()
            self.carDataPublisher.publish(array_msg)
            print("Publishing QCar data to ROS took: ", (time.time()-t0)*1000," ms")

def main():
    rclpy.init()
    dataPublisher_ = QCarDataPublisher()
    
    # Create threads for cyclicReceiveTCP and sendCameraData
    receive_thread = threading.Thread(target=dataPublisher_.cyclicReceiveTCP)
    
    # Start the threads
    receive_thread.start()

    rclpy.spin(dataPublisher_)
    
    # Wait for the threads to finish
    receive_thread.join()
    rclpy.shutdown()

main()