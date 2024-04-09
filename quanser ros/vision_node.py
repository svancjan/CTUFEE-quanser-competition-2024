import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import time
import sys
import markers_detection as md
import pickle

from Deserializer import deserialize_message, serialize_message

class vision_node(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.vision_publisher = self.create_publisher(ByteMultiArray, 'VisionData', qos_profile=qos_profile)

        self.subscription2 = self.create_subscription(
            ByteMultiArray,
            'CameraRGB',
            self.processData,
            qos_profile = qos_profile,
            raw=True
        )

    def processData(self, message):
        # needs to get image[0] becaise it returns tunple (image, timestamp)
        msg = deserialize_message(message)
        image = msg[0]
        timstamp = msg[1]
        print("Received msg type: ", type(image))

        # if data are image do this:

        bb_stop = md.detect_stop_sign(image)
        bb_traffic = md.detect_traffic_lights(image)

        # serialize data and publish them
        msg = serialize_message((bb_stop, bb_traffic))
        self.vision_publisher.publish(msg)
        
def __main__():
    rclpy.init()
    vision = vision_node()
    rclpy.spin(vision)
    
__main__()
        






