import numpy as np
from std_msgs.msg import ByteMultiArray
from win_interface.tcp_manager import TCPPublisher, TCPSubscriber
import Deserializer

import rclpy
from rclpy.node import Node

import pickle

class QCarDataPublisher(Node):
    def __init__(self) -> None:
        super().__init__('qcarDataPublisher')

        # Create TCPManager instance
        self.tcpPublisher = TCPPublisher(5555)
        self.tcpCameraSubscriber = TCPSubscriber("169.254.165.200", 5556, bytes([0x80,0x04,0x95,0x87]))
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.rgbCameraPublisher = self.create_publisher(ByteMultiArray, 'CameraRGB', qos_profile = qos_profile)
        self.receiverTimer = self.create_timer(0.005, self.receiveTCP)
        self.publishCamTimer = self.create_timer(0.005, self.ROSpublishCameraData)
        
        self.controlSubscriber = self.create_subscription(
            ByteMultiArray,
            'LongitudinalPlanning',
            self.sendTrafficTCP,
            qos_profile = qos_profile,
            raw=True
        )
        
        self.controlSubscriber = self.create_subscription(
            ByteMultiArray,
            'TransversePlanning',
            self.sendLinesTCP,
            qos_profile = qos_profile,
            raw=True
        )

        self.cameraData = None
        print("Init done!")
        
    def receiveTCP(self):
        msg = self.tcpCameraSubscriber.receive_msg()
        if (msg is not None):
            self.cameraData = msg
            
    def ROSpublishCameraData(self):
        if rclpy.ok() and self.cameraData is not None:
            self.rgbCameraPublisher.publish(self.cameraData)
        
    def sendTrafficTCP(self, message):
        self.tcpPublisher.send_msg(message)
        
    def sendLinesTCP(self, message):
        self.tcpPublisher.send_msg(message)


def __main__():
    rclpy.init()
    dataPublisher_ = QCarDataPublisher()
    rclpy.spin(dataPublisher_)
    
__main__()

