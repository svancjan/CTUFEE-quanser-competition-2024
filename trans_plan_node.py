import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import ByteMultiArray
import pickle
import transverse_planner as tp


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

class trans_plan_node(Node):
    def __init__(self):
        super().__init__('trans_plan_node')

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub = self.create_publisher(ByteMultiArray, 'CenterLine', qos_profile=qos_profile)

        self.sub = self.create_subscription(
            ByteMultiArray,
            'TransversePlanning',
            self.processData,
            qos_profile = qos_profile,
            raw=True
        )
    
    def processData(self, message):
        """
        - in case both lines are received (yellow line and road line), center_line is calculated
          average of those two lines
        - in case only one of those lines is received, an offset curve is calculated, which is 
          half a car's width + 10 % from the given line
        - in case none of the lines is provided, then None is returned
        """
        lines = deserialize_message(message)
        center_line = tp.get_center_line(lines)

        msg = serialize_message(center_line)
        self.pub.publish(msg)


