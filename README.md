# CTUFEE-quanser-competition-2024

# Overall Solution

Our solution was developed to have the best coherence of performance and robustness to fulfil the rules of the first stage of the competition. During development, we put great emphasis on the modularity of the solution to be as transferable and versatile as possible. In anticipation of the second stage of the competition, which employs a real Qcar operating on a Linux system, we have designed our software accordingly. With the intention of utilizing the ROS 2 environment during this stage, we have established a connection layer that enables the use of a Linux PC equipped with the ROS 2 environment. This strategic approach ensures our preparedness for the forthcoming challenges of the competition. To simulate the Qcar we are running the Quanser interactive labs on a Windows PC and image processing and local planning. In Figure 1 there is a simplified graph of our solution.


![MicrosoftTeams-image](https://github.com/svancjan/CTUFEE-quanser-competition-2024/assets/161430370/22409b05-5606-40c3-9b89-9fe55b5bd2bb)
Figure 1:

As seen in Fig. 1 control algorithms are run on Windows PC along the Qcar simulation to maximize response time and minimize transport delay.  

# Object detection and localization

Our system is capable of identifying stop signs and calculating the corresponding distance from the vehicle. Additionally, it can discern traffic lights, ascertain their status (red/green), and determine the distance from the vehicle. To enhance our global plan we detect the right side of the road as well as the middle yellow line. These data are then used to control the correct position of the car on the global path.  We use the OpenCV library for detection in the first stage of the competition. We trained a neural network for detection. We tested the utilization of the NN during the development of the solution for the first stage. However, we decided not to use it in the first stage. This does not limit us in the use of just CUDA-compatible hardware.  For the second stage, we are planning on deploying the NN, since we got familiar with it and we already created automated procedures for dataset creation and labelling as well as training.

# Control
Our control solution is divided into two subclasses: one dedicated to longitudinal control and the other to lateral control.  As shown in the diagram in Figure 1 the classes are connected as the lateral control provides a reference signal for the longitudinal controller. Longitudinal control is implemented as a PID controller controlling the throttle position. The Lateral controller uses a hierarchical architecture, as shown in Figure 2. The reference for the controllers are, similarly as in Stanley Lateral Controller, the cross-track error and the heading error. A look-ahead component is also used for improving the path-tracking performance.


![HA](https://github.com/svancjan/CTUFEE-quanser-competition-2024/assets/153733241/fa12ea1c-930c-4441-afa7-bae1fed477e1)
Figure 2:


