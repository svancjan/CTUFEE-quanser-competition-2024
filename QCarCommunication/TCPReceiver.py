from win_interface.tcp_manager import TCPManager
import yaml

import time
import numpy as np

if __name__ == "__main__":
    # Load configuration from YAML file
    with open("tcpConfiguration.yaml", "r") as file:
        config = yaml.safe_load(file)

    # Create TCPManager instance
    tcp_manager = TCPManager(config["QCarIP"], config["CarToROSPort"], config["ROSToCarPort"])

    # Start receiving data
    tStart = time.time()
    timeList = list()
    
    while True:
        t0 = time.time()
        message = tcp_manager.receive_msg()
        if (message is not None):
            #if type(message) == dict: # corresponds to car data received
            #    #for key, value in message.items():
            #    #    print(key, type(value))
            tcp_manager.send_msg((0.1,np.pi/30))
            #elif type(message) == tuple:
            #    #print(message[0], message[1].shape)     
            t = time.time()-t0
            timeList.append(t)       
        
        if time.time() - tStart > 15:
            break
        
    print(max(timeList),
          sum(timeList)/len(timeList))
        
    tcp_manager.terminate()