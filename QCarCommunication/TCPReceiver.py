from win_interface.tcp_manager import TCPManager
import yaml

import time

if __name__ == "__main__":
    # Load configuration from YAML file
    with open("tcpConfiguration.yaml", "r") as file:
        config = yaml.safe_load(file)

    # Create TCPManager instance
    tcp_manager = TCPManager(config["QCarIP"], config["CarToROSPort"], config["ROSToCarPort"])

    # Start receiving data
    tcp_manager.start_receiving()
    t0Car = time.time()
    t0Cam = time.time()
    tStart = time.time()
    while True:
        if type(tcp_manager.latest_data) == dict: # corresponds to car data received
            #for key, value in tcp_manager.latest_data.items():
                #print(key, type(value))
            t0Car = time.time()
        else:
            t0Cam = time.time()
            #print(tcp_manager.latest_data[0], tcp_manager.latest_data[1].shape)
        print(time.time()-t0Car, time.time()-t0Cam)
        
        if time.time() - tStart > 30:
            break
        
    tcp_manager.terminate()