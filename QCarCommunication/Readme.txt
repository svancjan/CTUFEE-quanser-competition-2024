qlabs_setup.py and vehicle_control.py is used only for testing the readout on the vehicle, while running.
At the moment, QCarReadout is created inside vehicle_control.py and TCPReceiver is just testing object
for data receiving. TCPReceiver will be object containing ROS2, which distributes data to other classes.
In yaml file tcpConfiguration.yaml, IP address of the devices has to be set. If you are using same device,
please set both ROSIP and QCarIP to "localhost". If you are using 2 devices in same network, call "ipconfig"
in windows command line to get device IP address or "ifconfig" in the linux device.
