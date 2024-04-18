import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import numpy as np
import markers_detection as md
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

        self.long_planning_pub = self.create_publisher(ByteMultiArray, 'LongitudinalPlanning', qos_profile=qos_profile)

        self.trans_planning_pub = self.create_publisher(ByteMultiArray, 'TransversePlanning', qos_profile=qos_profile)

        self.subscription2 = self.create_subscription(
            ByteMultiArray,
            'CameraRGB',
            self.processData,
            qos_profile = qos_profile,
            raw=True
        )

    def processData(self, message):
        '''
        This function processes the image data received from the camera and
        detects stop signs and traffic lights, side of the road and yelllow line. It then publishes the data
        to the LongitudinalPlanning and TransversePlanning topics.

        Longitudinal Planning publisher data: (stop_distance, traffic_distance, traffic_state)
            if stop sign not detetcted, stop_distance = None
            if traffic light not detected, traffic_distance = None
            if detected distances is of type float
            traffic_state: int <-- state of the traffic light, 0 for red, 1 for green, 2 for yellow/unknown


        Transverse Planning publisher data: (road_line, yellow_line)
            if road line not detected, road_line = None
            if yellow line not detected, yellow_line = None
            if detected lines are of type list of tuples with shape (n,2), where n is the number of points 
        '''
        # needs to get image[0] because it returns tunple (image, timestamp)
        image = deserialize_message(message)[0]
        time_stamp = deserialize_message(message)[1]

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Longitudinal Planning ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        # detect stop sign and traffic light
        bb_stop = md.detect_stop_sign(image)
        bb_traffic = md.detect_traffic_lights(image)

        if bb_stop is not None:
            # if stop sign detected, get distance
            x, y, w, h, area = bb_stop
            stop_distance = np.mean(md.estimate_distance_y2(x, y, x+w, y+h, image,1))
        else:
            stop_distance = None
        
        if bb_traffic is not None:
            # if traffic light detected, get distance
            bb, traffic_state = bb_traffic
            x, y, w, h, area = bb
            traffic_distance = np.mean(md.estimate_distance_y2(x, y, x+w, y+h, image,0))
        else:
            traffic_distance = None
            traffic_state = None
        
        # serialize data and publish them
        msg = serialize_message((stop_distance, traffic_distance, traffic_state))
        self.long_planning_pub.publish(msg)

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Transverse Planning ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        yellow_line = md.detect_yellow_line(image)
        road_line = md.detect_road(image)

        msg = serialize_message((road_line,yellow_line))
        self.trans_planning_pub.publish(msg)



        






