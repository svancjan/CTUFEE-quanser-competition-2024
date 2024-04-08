import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import time
import sys

from win_interface import Deserializer

class ROSTopicSubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create a subscriber to the desired topic
        self.subscription1 = self.create_subscription(
            ByteMultiArray,
            'CarData',
            self.processData,
            qos_profile = qos_profile,
            raw=True
        )
        
        self.subscription2 = self.create_subscription(
            ByteMultiArray,
            'CameraRGB',
            self.processData,
            qos_profile = qos_profile,
            raw=True
        )
        
        self.last = time.perf_counter()
    
    def processData(self, message):
        print("Received msg type: ", type(Deserializer.deserialize_message(message)),". Time since last msg: ", (time.perf_counter() - self.last)*1000, " ms")
        self.last = time.perf_counter()

def main():
    rclpy.init()
    node = ROSTopicSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

main()