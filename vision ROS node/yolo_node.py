import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import numpy as np
from ultralytics import YOLO
import markers_detection as md
import cv2
import pickle
import json

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

class yolo_node(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.long_planning_pub = self.create_publisher(ByteMultiArray, 'transverseData', qos_profile=qos_profile)

        self.subscription2 = self.create_subscription(
            ByteMultiArray,
            'CameraRGB',
            self.processData,
            qos_profile = qos_profile,
            raw=True
        )
        with open('RS_road_h.json', 'r') as f:
            self.H_real_sense = np.array(json.load(f))

        # load model
        self.model = YOLO("yolov8n-seg.yaml")  # build a new model from scratch
        self.model = YOLO("weights.pt")

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

        # detect transverse
        result = self.model.predict(image, classes=[6])[0]
        if result.masks is None:
            # NO TRANSVERSE DETECTED
            transverse_data = None
        else:
            # TRANSVERSE DETECTED
            mask = result.masks.data.cpu().numpy()
            mask = mask[0, :, :]
            # 
            contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contour = contours[0]
            M = cv2.moments(contour)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            point = np.array([cx,cy]).reshape(1,2)
            # for contour in contours:
            #     M = cv2.moments(contour)
            #     if M["m00"] != 0:
            #         cX = int(M["m10"] / M["m00"])
            #         cY = int(M["m01"] / M["m00"])
            #         centers.append((cX, cY))
            car_coords_point = md.make_car_coords(point, self.H_real_sense)
            transverse_data = np.array(car_coords_point)

        msg = serialize_message(transverse_data)
        self.long_planning_pub.publish(msg)




        






