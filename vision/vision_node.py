import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import time
import sys
import distance_computation as dc
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
        image = deserialize_message(message)[0]
        stop_detected = False
        traffic_detected = False

        # if data are image do this:
        bb_stop = dc.detect_stop_sign(image)
        bb_traffic = dc.detect_traffic_lights(image)

        if bb_stop is not None:
            stop_detected = True
            # if stop sign detected, get distance
            x, y, w, h, area = bb_stop
            stop_distance = dc.estimate_distance_y2(x, y, x+w, y+h, image,1)
        
        if bb_traffic is not None:
            traffic_detected = True
            # if traffic light detected, get distance
            bb, state = bb_traffic
            x, y, w, h, area = bb
            traffic_distance = dc.estimate_distance_y2(x, y, x+w, y+h, image,0)
        
        # serialize data and publish them
        msg = serialize_message((stop_detected,traffic_detected,stop_distance, traffic_distance))
        self.vision_publisher.publish(msg)
        






