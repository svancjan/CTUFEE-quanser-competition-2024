import threading
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, ByteMultiArray, MultiArrayLayout, MultiArrayDimension
from .win_interface.tcp_manager import TCPManager
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
        
        with open("install/interfaceNode/lib/interfaceNode/tcpConfiguration.yaml", "r") as file:
            config = yaml.safe_load(file)

        # Create TCPManager instance
        self.tcp_manager = TCPManager(config["QCarIP"], config["CarToROSPort"], config["ROSToCarPort"])
        self.carData = dict()
        self.camerasData = dict()
        
        self.carDataLock = threading.Lock()
        self.camerasDataLock = threading.Lock()
        
        self.rosPublisher = self.create_publisher(ByteMultiArray, 'CameraRGB', 1)
        self.rosTimer = self.create_timer(0.01, self.send_array)
    
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
        
    def send_array(self):
        if rclpy.ok():
            # Convert the numpy array to a Image message
            array_msg = ByteMultiArray()
            if("realSenseRGBData" in self.camerasData.keys()):
                self.camerasDataLock.acquire()
                t0 = time.time()
                #layout = MultiArrayLayout()
                #layout.dim.append(MultiArrayDimension())
                #layout.dim[0].size = 921600 # 640*480*3
                #array_msg.data = {bytes(self.camerasData["realSenseRGBData"])}
                array_msg.data = {pickle.dumps(self.camerasData["realSenseRGBData"],protocol=pickle.DEFAULT_PROTOCOL)}
                print("assign:", time.time()-t0, sys.getsizeof(array_msg.data))
                a = array_msg.data.pop()
                print("arr:", time.time()-t0, pickle.loads(a).shape)
                self.camerasDataLock.release()
            # Publish the array message
            self.rosPublisher.publish(array_msg)

def main():
    rclpy.init()
    dataPublisher_ = QCarDataPublisher()
    
    # Create threads for cyclicReceiveTCP and send_array
    receive_thread = threading.Thread(target=dataPublisher_.cyclicReceiveTCP)
    
    # Start the threads
    receive_thread.start()

    rclpy.spin(dataPublisher_)
    
    # Wait for the threads to finish
    receive_thread.join()
    rclpy.shutdown()
