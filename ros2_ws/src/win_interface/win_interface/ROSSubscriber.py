import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import time
import sys

from win_interface import Deserializer
import pickle

class ROSTopicSubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.controlPublisher = self.create_publisher(ByteMultiArray, 'CarControl', qos_profile = qos_profile)
        self.controlTimer = self.create_timer(0.02, self.publishData)

        # Create a subscriber to the desired topic
        self.subscription1 = self.create_subscription(
            ByteMultiArray,
            'CarData',
            self.subscribeCar,
            qos_profile = qos_profile,
            raw=True
        )
        
        self.subscription2 = self.create_subscription(
            ByteMultiArray,
            'CameraRGB',
            self.subscribeCamera,
            qos_profile = qos_profile,
            raw=True
        )
        
        self.carData = None
        self.camData = None
        self.controlData = None
        
        self.last = time.perf_counter()
        print("Init done!")
    
    def subscribeCar(self, message):
        self.last = time.perf_counter()
        self.carData = Deserializer.deserialize_message(message)
        
    def subscribeCamera(self, message):
        self.camData = Deserializer.deserialize_message(message)
        
    def publishData(self):
        if self.carData is not None:
            self.controlData = (0.2, 0.1, self.carData["timeStamp"])
            data = pickle.dumps(self.controlData, protocol = pickle.DEFAULT_PROTOCOL)
            self.controlPublisher.publish(data)

def __main__():
    rclpy.init()
    node = ROSTopicSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

__main__()
