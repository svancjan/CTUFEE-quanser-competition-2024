import numpy as np
from std_msgs.msg import ByteMultiArray
from .tcp_manager import TCPPublisher, TCPSubscriber

import rclpy
from rclpy.node import Node

from ..Deserializer import Deserializer

import pickle

class QCarDataPublisher(Node):
    def __init__(self) -> None:
        super().__init__('qcarDataPublisher')

        # Create TCPManager instance
        self.tcpPublisherLines = TCPPublisher(5555)
        self.tcpPublisherSigns = TCPPublisher(5554)
        self.tcpPublisherNeuralNetwork = TCPPublisher(5553)
        self.tcpCameraSubscriber = TCPSubscriber("169.254.165.200", 5556)
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.rgbCameraPublisher = self.create_publisher(ByteMultiArray, 'CameraRGB', qos_profile = qos_profile)
        
        self.receiverTimer = self.create_timer(0.005, self.receiveTCP)
        
        # stop and traffic lights
        self.LongitudalPlanningSub = self.create_subscription(
            ByteMultiArray,
            'LongitudinalPlanning',
            self.sendTrafficTCP,
            qos_profile = qos_profile,
            raw=True
        )
        
        # lateral planning
        self.TransversePlanningSub = self.create_subscription(
            ByteMultiArray,
            'CenterLine',
            self.sendLinesTCP,
            qos_profile = qos_profile,
            raw=True
        )
        
        
        self.NNDataSub = self.create_subscription(
            ByteMultiArray,
            'NNData',
            self.sendNeuralNetworkData,
            qos_profile = qos_profile,
            raw=True
        )
        self.XXX = None
        self.cameraData = None
        self.get_logger().info("Simulation interface node initialization done!")
        
    def __del__(self):
        self.get_logger().info("Simulation interface node shutdown!")
        
    def receiveTCP(self):
        msg = self.tcpCameraSubscriber.receive_msg()
        if (msg is not None):
            self.rgbCameraPublisher.publish(msg)
        
    def sendTrafficTCP(self, message):
        self.tcpPublisherSigns.send_msg(message)
        
    def sendLinesTCP(self, message):
        self.tcpPublisherLines.send_msg(message)
        
    def sendNeuralNetworkData(self, message):
        self.tcpPublisherNeuralNetwork.send_msg(message)


def __main__():
    rclpy.init()
    dataPublisher_ = QCarDataPublisher()
    rclpy.spin(dataPublisher_)
    
__main__()

