import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import numpy as np
import pickle
import json
import cv2
import time

from ..Deserializer import Deserializer
from ..markers_detection import *

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
        with open(os.path.dirname(os.path.realpath(__file__)) + '/../RS_road_h.json', 'r') as f:
            self.H_real_sense = np.array(json.load(f))
            
        self.get_logger().info("Vision node initialization done!")
        
        self.test = True
        
    def __del__(self):
        self.get_logger().info("Vision node shutdown!")

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
        t0 = time.time()
        msg = Deserializer.deserialize_message(message)
        image = msg[0]
        vehicle_position = msg[1]
        time_stamp = msg[2]
        
        #cv2.imwrite("homo.jpg", image)

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Longitudinal Planning ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        # detect stop sign and traffic light
        bb_stop = detect_stop_sign(image)
        bb_traffic = detect_traffic_lights(image)

        if bb_stop is not None:
            # if stop sign detected, get distance
            x, y, w, h, area = bb_stop
            stop_distance = np.mean(estimate_distance_y2(x, y, x+w, y+h, image,1))
            stop_position = vehicle_position[:2] + np.array([np.cos(vehicle_position[2]), np.sin(vehicle_position[2])]) * (stop_distance-0.4)
        else:
            stop_position = None
        
        if bb_traffic is not None:
            # if traffic light detected, get distance
            bb, traffic_state = bb_traffic
            x, y, w, h, area = bb
            traffic_distance = np.mean(estimate_distance_y2(x, y, x+w, y+h, image,0))
            traffic_position = vehicle_position[:2] + np.array([np.cos(vehicle_position[2]), np.sin(vehicle_position[2])]) * (traffic_distance - 1.65)
        else:
            traffic_position = None
            traffic_state = None
        
        # Print objects to the FPV image
        font = cv2.FONT_HERSHEY_SIMPLEX
        textPos = (50,50)
        fontscale = 1
        color = (0,0,0)
        thickness = 3
        image = cv2.putText(image, str(round(time_stamp-1 if time_stamp > 1 else 0,2)), textPos, cv2.FONT_HERSHEY_SIMPLEX, fontscale, color, thickness)
        
        if (bb_stop is not None):
            x, y, w, h, area = bb_stop
            image = cv2.rectangle(image, (x,y), (x+w,y+h), (255,0,255), 2)
            textSize = cv2.getTextSize(str("Stop"),font, 1, 2)[0]
            textX = int(x+w/2-textSize[0]/2)
            image = cv2.putText(image, str("Stop"), (textX,y), font, 1, (255,0,255), 2)
        if (bb_traffic is not None):
            x, y, w, h, area = bb
            text = "Traffic"
            if traffic_state == 0:
                text = "Red"
                color = (0,0,255)
            else:
                text = "Green"
                color = (0,255,0)
            image = cv2.rectangle(image, (x,y), (x+w, y+h), color, 2)
            textSize = cv2.getTextSize(text, font, 1, 2)[0]
            textX = int(x+w/2-textSize[0]/2)
            image = cv2.putText(image, text, (textX,y), font, 1, color, 2)
        
        #cv2.imwrite("/home/kodytota/Desktop/rosws_Quanser/drive/" + str(int(time_stamp*1000)) + ".jpg", image)
        
        # serialize data and publish them
        msg = Deserializer.serialize_message((stop_position, traffic_position, traffic_state))
        self.long_planning_pub.publish(msg)

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Transverse Planning ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        yellow_line = detect_yellow_line(image, self.H_real_sense)
        road_line = detect_road(image, self.H_real_sense)

        msg = Deserializer.serialize_message((road_line, yellow_line, vehicle_position))
        self.trans_planning_pub.publish(msg)
        self.get_logger().info("Vision time:" + str(time.time()-t0))
        
        
def __main__():
    rclpy.init()
    visionNode_ = vision_node()
    rclpy.spin(visionNode_)
    
__main__()


        






