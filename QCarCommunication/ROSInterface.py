import threading
import rclpy
import numpy as np
from sensor_msgs.msg import Image
from win_interface.tcp_manager import TCPManager
import yaml
from TCPReceiver import TCPReceiver

class QCarDataPublisher():
    def __init__(self) -> None:
        self.shutDownRequired = False
        
        with open("tcpConfiguration.yaml", "r") as file:
            config = yaml.safe_load(file)

        # Create TCPManager instance
        self.tcp_manager = TCPManager(config["QCarIP"], config["CarToROSPort"], config["ROSToCarPort"])
        self.carData = dict()
        self.camerasData = dict()
        
        self.carDataLock = threading.Lock()
        self.camerasDataLock = threading.Lock()
        
    def cyclicReceiveTCP(self):        
        while True:
            msg = self.tcp_manager.receive_msg()
            if (msg is not None):
                if (msg["Type"] == "CarData"):
                    self.carDataLock.acquire()
                    self.carData = msg
                    self.carDataLock.release()
                elif (msg["Type"] == "CamerasData"):
                    self.camerasDataLock.acquire()
                    self.camerasData = msg
                    self.camerasDataLock.release()
            continue
        
    def send_array(self):
        # Initialize the ROS node
        rclpy.init()

        # Create a publisher to send the numpy array
        array_pub = rclpy.create_publisher(Image, 'CameraRGB', 0)

        # Create a ROS rate object to control the publishing rate
        rate = rclpy.Rate(100)  # 100 Hz

        while (rclpy.ok() or self.shutDownRequired):
            # Convert the numpy array to a Image message
            array_msg = Image()
            
            if("realSenseRGBData" in self.camerasData.keys()):
                self.camerasDataLock.acquire()
                array_msg.data = self.camerasData["realSenseRGBData"].flatten().tolist()
                self.camerasDataLock.release()
            # Publish the array message
            array_pub.publish(array_msg)

            # Sleep to maintain the desired publishing rate
            rate.sleep()

if __name__ == '__main__':
    dataPublisher_ = QCarDataPublisher()
    
    # Create threads for cyclicReceiveTCP and send_array
    receive_thread = threading.Thread(target=dataPublisher_.cyclicReceiveTCP)
    send_thread = threading.Thread(target=dataPublisher_.send_array)
    
    # Start the threads
    receive_thread.start()
    send_thread.start()
    
    # Wait for the threads to finish
    receive_thread.join()
    send_thread.join()
