import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import time

import Deserializer

class ROSTopicSubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')

        # Create a subscriber to the desired topic
        self.subscription1 = self.create_subscription(
            ByteMultiArray,
            'CarData',
            self.processData,
            1,
            raw=True
        )
        
        self.subscription2 = self.create_subscription(
            ByteMultiArray,
            'CameraRGB',
            self.processData,
            1,
            raw=True
        )
        
        self.subscription3 = self.create_subscription(
            ByteMultiArray,
            'CameraDepth',
            self.processData,
            1,
            raw=True
        )
        
        self.subscription4 = self.create_subscription(
            ByteMultiArray,
            'CameraFront',
            self.processData,
            1,
            raw=True
        )
        
        self.last = time.time()
    
    def processData(self, message):
        print("Received msg type: ", type(Deserializer.deserialize_message(message)),". Time since last msg: ", (time.time() - self.last)*1000, " ms")
        self.last = time.time()

def main():
    rclpy.init()
    node = ROSTopicSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

main()