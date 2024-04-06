from win_interface.tcp_manager import TCPManager
import yaml

import time
import numpy as np

import psutil, os

if __name__ == "__main__":
    
    p = psutil.Process(os.getpid())
    p.nice(psutil.HIGH_PRIORITY_CLASS)
    
    # Load configuration from YAML file
    with open("tcpConfiguration.yaml", "r") as file:
        config = yaml.safe_load(file)

    # Create TCPManager instance
    tcp_manager = TCPManager(config["QCarIP"], config["CarToROSPort"], config["ROSToCarPort"])

    # Start receiving data
    tStart = time.time()
    timeList = list()
    
    timeControl = list()
    conT0 = None
    
    while True:
        message = tcp_manager.receive_msg(10)
        if (message is not None):
            if type(message) == dict: # corresponds to car data received
                tcp_manager.send_msg((0.1, np.pi/30, message["timeStamp"]))
                if conT0 is not None:
                    timeControl.append(time.perf_counter()-conT0)
                conT0 = time.perf_counter()   
            elif type(message) == tuple:
                continue
                #print(message[0], message[1]) 
        if time.time() - tStart > 25:
            break
        
    print(max(timeControl),
          sum(timeControl)/len(timeControl))
    print("{:<8} {:<8} {:<8} ".format('name','max','mean'))
    print("{:<8} {:<8} {:<8} ".format(
        "car","{:.1f}".format(max(timeControl)*1000), 
          "{:.1f}".format(sum(timeControl)/(len(timeControl))*1000)))
        
    tcp_manager.terminate()