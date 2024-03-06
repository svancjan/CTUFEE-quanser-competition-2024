from win_interface.tcp_manager import TCPManager
import yaml

if __name__ == "__main__":
    # Load configuration from YAML file
    with open("tcpConfiguration.yaml", "r") as file:
        config = yaml.safe_load(file)

    # Create TCPManager instance
    tcp_manager = TCPManager(config["QCarIP"], config["CarToROSPort"], config["ROSToCarPort"])

    # Start receiving data
    tcp_manager.start_receiving()
    while True:
        print(tcp_manager.latest_data, end="\r")
        continue