import threading
import numpy as np
from std_msgs.msg import  ByteMultiArray, Float32
from win_interface.tcp_manager import TCPPublisher, TCPSubscriber
import time

import rclpy
from rclpy.node import Node

import pickle

def serialize_message(message):
    try:
        message = pickle.dumps(message)
        return message
    except pickle.PicklingError as e:
        print(f"Error occurred while serializing message: {e}")
        return None

def deserialize_message(message):
    try:
        message = pickle.loads(message)
        return message
    except pickle.UnpicklingError as e:
        print(f"Error occurred while deserializing message: {e}")
        return None

class QCarDataPublisher(Node):
    def __init__(self) -> None:
        super().__init__('qcarDataPublisher')

        # Create TCPManager instance
        self.tcpPublisher = TCPPublisher(5555)
        self.tcpCarSubscriber = TCPSubscriber("169.254.165.200", 5556, bytes([0x80,0x04,0x95,0x6c]))
        self.tcpCameraSubscriber = TCPSubscriber("169.254.165.200", 5556, bytes([0x80,0x04,0x95,0x87]))
        self.carData = None
        self.cameraData = None
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.carDataPublisher = self.create_publisher(ByteMultiArray, 'CarData', qos_profile = qos_profile)
        self.rgbCameraPublisher = self.create_publisher(ByteMultiArray, 'CameraRGB', qos_profile = qos_profile)
        self.cameraTimer = self.create_timer(0.05, self.ROSpublishCameraData)
        self.carTimer = self.create_timer(0.005, self.ROSpublishCarData)
        self.receiverTimer = self.create_timer(0.005, self.cyclicReceiveTCP)
        
        self.controlRequest = [0] * 4
        self.controlSubscriber = self.create_subscription(
            ByteMultiArray,
            'CarControl',
            self.sendControlTCP,
            qos_profile = qos_profile,
            raw=True
        )
        
        self.longitudalControlSub = self.create_subscription(
            Float32,
            'CarControl_Longitudal',
            self.receiveLongitudal,
            qos_profile = qos_profile,
            raw = False
        )
            
        self.lateralControlSub = self.create_subscription(
            Float32,
            'CarControl_Lateral',
            self.receiveLateral,
            qos_profile = qos_profile,
            raw = False
        )
        self.counter = 0
        print("Init done!")
        
    def cyclicReceiveTCP(self):
        msg = self.tcpCarSubscriber.receive_msg()
        if (msg is not None):
            self.carData = msg
        msg = self.tcpCameraSubscriber.receive_msg()
        if (msg is not None):
            self.cameraData = msg
    
    def ROSpublishCarData(self):
        if rclpy.ok() and self.carData is not None:
            self.carDataPublisher.publish(self.carData)
            
    def ROSpublishCameraData(self):
        if rclpy.ok() and self.cameraData is not None:
            self.rgbCameraPublisher.publish(self.cameraData)
                       
    def receiveLongitudal(self, message):
        self.controlRequest[0] = message.data
    	
    def receiveLateral(self, message):
        self.controlRequest[1] = message
        
    def sendControlTCP(self, message):
        self.controlRequest[2] = deserialize_message(message)[2]
        self.controlRequest[3] = self.counter
        self.counter = self.counter + 1
        self.tcpPublisher.send_msg(serialize_message(self.controlRequest)) # if raw


def __main__():
    rclpy.init()
    dataPublisher_ = QCarDataPublisher()
    rclpy.spin(dataPublisher_)
    
__main__()

