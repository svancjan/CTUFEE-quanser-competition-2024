import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import numpy as np
import pickle
from shapely import LineString
import json
import time

from ..Deserializer import Deserializer

CAR_WIDTH = 0.24

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
            if len(points) > 5:
                points = points[1::5, :]
        else:
            return None
        
        # Create a LineString from the points
        line = LineString(points)
        
        # Offset the LineString
        offset_line = line.parallel_offset(distance, 'left',  join_style='round')

        offset_line = np.array(offset_line.coords)
         
        return offset_line
    

    def processData(self, message):
        t0 = time.time()
        msg = Deserializer.deserialize_message(message)
        road_line = msg[0]
        vehicle_position = msg[2]

        if road_line is not None:
            road_line = sorted(road_line, key=lambda x: np.linalg.norm(vehicle_position[:2]-x))
        
        # pi/2 rad is top, 0 is right globally
                      
            try:
                road_line = np.matmul(road_line,np.eye(2))
                offset_line = self.offset_curve_mine(road_line, CAR_WIDTH/2)
            except:
                offset_line = None
            
            if offset_line is not None:           
                fi = vehicle_position[2]-np.pi/2 # rotate clockwise
                rot_mat = np.array([[np.cos(fi), -np.sin(fi)],
        	    	      [np.sin(fi), np.cos(fi)]], dtype=np.float64)

                offset_line = np.matmul(rot_mat,np.transpose(offset_line))
                offset_line = offset_line + vehicle_position[:2].reshape((2,1))
                offset_line = np.transpose(offset_line)

                xpnext = np.roll(offset_line[:,0], -1)
                pnext = np.roll(offset_line[:,1], -1)

                angle = np.arctan2(pnext - offset_line[:,0], xpnext - offset_line[:,1])
                angle[-1] = angle[-2]
                plan = np.array([offset_line[:,0], offset_line[:,1], angle]).T
            else:
                plan = None

            self.trans_planning_pub.publish(Deserializer.serialize_message(plan))
        
def __main__():
    rclpy.init()
    lateralPlanNode_ = lateral_plan_node()
    rclpy.spin(lateralPlanNode_)
    
__main__()


        






