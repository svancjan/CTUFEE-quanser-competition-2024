import threading
import numpy as np
from std_msgs.msg import  ByteMultiArray
from win_interface.tcp_manager import TCPPublisher, TCPSubscriber
import time

import rclpy
from rclpy.node import Node

import pickle

class QCarDataPublisher(Node):
    def __init__(self) -> None:
        super().__init__('qcarDataPublisher')
        self.shutDownRequired = False

        # Create TCPManager instance
        self.tcpPublisher = TCPPublisher(5556)
        self.tcpCarSubscriber = TCPSubscriber("localhost", 5555, bytes([0x80,0x4,0x95,0x19]))
        self.tcpCameraSubscriber = TCPSubscriber("localhost", 5555, bytes([0x80,0x4,0x95,0x87]))
        self.carData = None
        self.cameraData = None
        
        self.carDataLock = threading.Lock()
        self.camerasDataLock = threading.Lock()
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.carDataPublisher = self.create_publisher(ByteMultiArray, 'CarData', qos_profile = qos_profile)
        self.rgbCameraPublisher = self.create_publisher(ByteMultiArray, 'CameraRGB', qos_profile = qos_profile)
        self.cameraTimer = self.create_timer(0.05, self.sendCameraData)
        self.carTimer = self.create_timer(0.005, self.sendCarData)
        self.receiverTimer = self.create_timer(0.005, self.cyclicReceiveTCP)
        
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
        
        self.carData = pickle.dumps(self.carData, protocol=pickle.DEFAULT_PROTOCOL)
        self.cameraData = pickle.dumps(self.cameraData, protocol=pickle.DEFAULT_PROTOCOL)
        self.camT0 = None
        self.carT0 = None
        self.carTimes = list()
        self.camTimes = list()
        self.recTimer = list()
        self.startTime = time.perf_counter()
            
    def cyclicReceiveTCP(self):
        t0 = time.perf_counter()
        if (time.perf_counter() > self.startTime + 5):
            print("Max time between car data messages: ", np.max(self.carTimes), " ms")
            print("Max time between camera data messages: ", np.max(self.camTimes), " ms")
            print("Max time of receiver loop: ", np.max(self.recTimer), " ms")
            
            import matplotlib.pyplot as plt
            plt.hist(self.carTimes)
            plt.show()
            rclpy.shutdown()
        msg = self.tcpCarSubscriber.receive_msg()
        if (msg is not None):
            self.carData = msg
        msg = self.tcpCameraSubscriber.receive_msg()
        self.cameraData = (np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8), time.perf_counter())
        self.cameraData = pickle.dumps(self.cameraData, protocol=pickle.DEFAULT_PROTOCOL)
        if (msg is not None):
            self.cameraData = msg
        self.recTimer.append((time.perf_counter()-t0)*1000)
                        
    def sendCameraData(self):
        if rclpy.ok() and self.cameraData is not None:
            self.rgbCameraPublisher.publish(self.cameraData)
            if self.camT0 is not None:
                self.camTimes.append((time.perf_counter()-self.camT0)*1000)
            self.camT0 = time.perf_counter()
            
            
    def sendCarData(self):
        if rclpy.ok() and self.carData is not None:
            self.carDataPublisher.publish(self.carData)
            if self.carT0 is not None:
                self.carTimes.append((time.perf_counter()-self.carT0)*1000)
            self.carT0 = time.perf_counter()
            
            
    def sendControlData(self, message):
        self.tcpPublisher.send_msg(message) # if raw

def main():
    rclpy.init()
    dataPublisher_ = QCarDataPublisher()
    
    # Create threads for cyclicReceiveTCP and sendCameraData
    #receive_thread = threading.Thread(target=dataPublisher_.cyclicReceiveTCP)
    
    # Start the threads
    #receive_thread.start()

    rclpy.spin(dataPublisher_)
    
    # Wait for the threads to finish
    #receive_thread.join()
    
#main()

import pstats, cProfile
cProfile.run('main()','restats')
stat = pstats.Stats('restats')
stat.sort_stats('cumulative').print_stats(20)