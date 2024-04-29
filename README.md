# CTUFEE-quanser-competition-2024

# Overall Solution

Our solution was developed to have the best performance and with suitable robustness in the first stage of the competition. During development, we put great emphasis on the solution to be as transferable to the second stage of competition. As we would like to use the ROS 2 environment in the second stage, we developed a link that made it possible to use a Linux PC with the ROS 2 environment installed. To simulate the Qcar we are running the Quanser interactive labs on a Windows PC. In Figure 1 there is a simplified graph of our solution.


![MicrosoftTeams-image](https://github.com/svancjan/CTUFEE-quanser-competition-2024/assets/161430370/22409b05-5606-40c3-9b89-9fe55b5bd2bb)


# Object detection and localization

We mainly use images from the Realsense camera. We can detect stop signs and their distance from the car, traffic lights, their state(red/green), and distance from the car. We detect the right side of the road and we detect the yellow line. In the simulation, we use the OpenCV library. We trained a neural network for detecting those and much more and we already tried during the development of the solution. However, we decided not to use it in the first stage. For the second stage, we are planning on deploying the neural network since we got familiar with it and we already created automated procedures for dataset creation and labelling as well as training.

# Control
to be written


# ROS 2 interface

