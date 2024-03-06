qlabs_setup.py and vehicle_control.py is used only for testing the readout on the vehicle, while running.
At the moment, QCarReadout is created inside vehicle_control.py and TCPReceiver is jsut testing object
for data receiving. TCPReceiver will be object containing ROS2, which distributes data to other classes.