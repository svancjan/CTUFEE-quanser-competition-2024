import threading
import numpy as np
from std_msgs.msg import  ByteMultiArray, MultiArrayLayout, MultiArrayDimension
from tcp_manager import TCPPublisher, TCPSubscriber
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
        self.tcpPublisher = TCPPublisher(config["ROSToCarPort"])
        self.tcpCarSubscriber = TCPSubscriber(config["QCarIP"], config["CarToROSPort"], bytes([0x80,0x4,0x95,0x19]))
        self.tcpCameraSubscriber = TCPSubscriber(config["QCarIP"], config["CarToROSPort"], bytes([0x80,0x4,0x95,0x87]))
        self.carData = None
        self.cameraData = None
        
        self.carDataLock = threading.Lock()
        self.camerasDataLock = threading.Lock()
        
        self.carDataPublisher = self.create_publisher(ByteMultiArray, 'CarData', qos_profile=1)
        self.rgbCameraPublisher = self.create_publisher(ByteMultiArray, 'CameraRGB', qos_profile=1)
        self.cameraTimer = self.create_timer(0.05, self.sendCameraData)
        self.carTimer = self.create_timer(0.005, self.sendCarData)
        
        self.controlSubscriber = self.create_subscription(
            ByteMultiArray,
            'CarControl',
            self.sendControlData,
            1,
            raw=True
        )
        
        self.cameraData = (np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8), time.perf_counter())
        self.carData = dict()
        self.carData["timeStamp"] = time.perf_counter()
        self.carData["batteryVoltage"] = np.random.uniform(10.0, 14.0)
        self.carData["motorSpeed"] = np.random.randint(0, 100)
        self.carData["temperature"] = np.random.uniform(20.0, 40.0)
        self.carData["position"] = np.random.uniform(0.0, 100.0, size=(3,))
        self.carData["orientation"] = np.random.uniform(0.0, 360.0, size=(3,))
        self.carData["lidarData"] = np.random.uniform(0.0, 100.0, size=(360,))
    
    '''def cyclicReceiveTCP(self):        
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
            continue'''
            
    def cyclicReceiveTCP(self):        
        while True:
            msg = self.tcpCarSubscriber.receive_msg()
            if (msg is not None):
                self.carDataLock.acquire()
                self.carData = msg
                self.carDataLock.release()
            msg = self.tcpCameraSubscriber.receive_msg()
            if (msg is not None):
                self.camerasDataLock.acquire()
                self.cameraData = msg
                self.camerasDataLock.release()
            continue
        
    def sendCameraData(self):
        t0 = time.perf_counter()
        if rclpy.ok() and self.cameraData is not None:
            self.rgbCameraPublisher.publish(self.cameraData)
            print("Publishing RS RGB data to ROS took: ", (time.perf_counter()-t0)*1000," ms")
            
    def sendCarData(self):
        t0 = time.perf_counter()
        if rclpy.ok() and self.carData is not None:
            self.carDataPublisher.publish(self.carData)
            print("Publishing QCar data to ROS took: ", (time.perf_counter()-t0)*1000," ms")
            
    def sendControlData(self, message):
        self.tcpPublisher.send_msg(message) # if raw

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