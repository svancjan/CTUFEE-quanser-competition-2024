# CTUFEE-quanser-competition-2024

# Overall Solution

Our solution was developed to have the best coherence of performance and robustness to fulfil the rules of the first stage of the competition. During development, we put great emphasis on the modularity of the solution to be as transferable and versatile as possible. In anticipation of the second stage of the competition, which employs a real Qcar operating on a Linux system, we have designed our software accordingly. With the intention of utilizing the ROS 2 environment during this stage, we have established a connection layer that enables the use of a Linux PC equipped with the ROS 2 environment. This strategic approach ensures our preparedness for the forthcoming challenges of the competition. To simulate the Qcar we are running the Quanser interactive labs on a Windows PC and image processing and local planning. In Figure 1 there is a simplified graph of our solution.

![quanser_diagram_fix2](https://github.com/svancjan/CTUFEE-quanser-competition-2024/assets/153294405/e4d36737-5a47-422e-bd23-2b1443b6c640)
Figure 1:

As seen in Fig. 1 control algorithms are run on Windows PC along the Qcar simulation to maximize response time and minimize transport delay.  

# Object detection and localization

Our system is capable of identifying stop signs and calculating the corresponding distance from the vehicle. Additionally, it can discern traffic lights, ascertain their status (red/green), and determine the distance from the vehicle. To enhance our global plan we detect the right side of the road as well as the middle yellow line. These data are then used to control the correct position of the car on the global path. Right at the moment the accuracy of the (lateral) local planner is not sufficient, so the resulting algorithms rely on (lateral) global plan only.  We use the OpenCV library for detection in the first stage of the competition. We trained a neural network for detection. We tested the utilization of the NN during the development of the solution for the first stage. However, we decided not to use it in the first stage. This does not limit us in the use of just CUDA-compatible hardware.  For the second stage, we are planning on deploying the NN, since we got familiar with it and we already created automated procedures for dataset creation and labelling as well as training.

# Control
Our control solution is divided into two subclasses: one dedicated to longitudinal control and the other to lateral control.  As shown in the diagram in Figure 1 the classes are connected as the lateral control provides a reference signal for the longitudinal controller. Longitudinal control is implemented as a PID controller controlling the throttle position. The Lateral controller uses a hierarchical architecture, as shown in Figure 2. The reference for the controllers are, similarly as in Stanley Lateral Controller, the cross-track error and the heading error. A look-ahead component is also used for improving the path-tracking performance.


![HA](https://github.com/svancjan/CTUFEE-quanser-competition-2024/assets/153733241/fa12ea1c-930c-4441-afa7-bae1fed477e1)
Figure 2:

# How to run
Whole program is divided into two workspaces. One has to contain Windows OS, which will run Quanser Interactive Labs and compute desired control action to the simulation (shall use QCarCommunication folder, then run file QCarReadout.py). Other workspace should preferably run on Linux OS. This workspace is in ROS2 environment and handles, vision, and lateral planning (runned via ros2 launch described below).
On both systems multiple libraries for python has to be installed. Except for standard libraries (numpy, etc.), there are others, which are present in the requirements.txt file. In the Linux machine, ROS2 environment has to be installed. After that, ROS2 has to be sourced, and built (colcon build --symlink-install). Then one has to source the local workspace, and run the program using prepared launch file (ros2 launch ctu_quanser_comp startup_launch.py).
Static IP addresses are used for the communication via local network (tested with ethernet). Linux machine has IP address 169.254.165.100, Windows 169.254.165.200, both netmasks are 255.255.0.0.
