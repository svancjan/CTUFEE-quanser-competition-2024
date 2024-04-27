import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import numpy as np
import pickle
from shapely import LineString
import json

from ..Deserializer import Deserializer

CAR_WIDTH = 0.21

class lateral_plan_node(Node):
    def __init__(self):
        super().__init__('lateral_plan_node')
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.trans_planning_pub = self.create_publisher(ByteMultiArray, 'CenterLine', qos_profile=qos_profile)

        self.subscription2 = self.create_subscription(
            ByteMultiArray,
            'TransversePlanning',
            self.processData,
            qos_profile = qos_profile,
            raw=True
        )
            
        self.get_logger().info("Lateral planning node initialization done!")
        
    def __del__(self):
        self.get_logger().info("Lateral planning node shutdown!")
            
    def offset_curve_mine(self, points, distance):
        #take each 5th point
        if points is not None:
            points = points[1::5, :]
        else:
            return None

        # Create a LineString from the points
        line = LineString(points)

        # Offset the LineString
        offset_line = line.parallel_offset(distance, 'left',  join_style='round')

        offset_line = np.array(offset_line.coords)

        # Return the coordinates of the offset curve as a numpy array
        return offset_line

    def processData(self, message):
        msg = Deserializer.deserialize_message(message)
        road_line = msg[0]
        yellow_line = msg[1]
        vehicle_position = msg[2]
        
        road_line = sorted(road_line, key=lambda x: np.linalg.norm(vehicle_position[:2]-x))
        
        # pi/2 rad is top, 0 is right, i want my matrix to rotate opposite
        fi = vehicle_position[2]-np.pi/2 # rotate clockwise
        rot_mat = np.array([[np.cos(fi), -np.sin(fi)],
        		      [np.sin(fi), np.cos(fi)]], dtype=np.float64)
        		      
        road_line = np.matmul(rot_mat,np.transpose(road_line))
        road_line = road_line + vehicle_position[:2].reshape((2,1))
        road_line = np.transpose(road_line)
            
        offset_line = self.offset_curve_mine(road_line, CAR_WIDTH/2)
        
        self.trans_planning_pub.publish(Deserializer.serialize_message(offset_line))
        
def __main__():
    rclpy.init()
    lateralPlanNode_ = lateral_plan_node()
    rclpy.spin(lateralPlanNode_)
    
__main__()


        






