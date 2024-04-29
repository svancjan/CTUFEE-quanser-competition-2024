# CTUFEE-quanser-competition-2024

# Overall Solution

Our solution was developed to have the best coherence of performance and robustness to fulfil the rules of the first stage of the competition. During development, we put great emphasis on the modularity of the solution to be as transferable and versatile as possible. In anticipation of the second stage of the competition, which employs a real Qcar operating on a Linux system, we have designed our software accordingly. With the intention of utilizing the ROS 2 environment during this stage, we have established a connection layer that enables the use of a Linux PC equipped with the ROS 2 environment. This strategic approach ensures our preparedness for the forthcoming challenges of the competition. To simulate the Qcar we are running the Quanser interactive labs on a Windows PC and image processing and local planning. In Figure 1 there is a simplified graph of our solution.


![MicrosoftTeams-image](https://github.com/svancjan/CTUFEE-quanser-competition-2024/assets/161430370/22409b05-5606-40c3-9b89-9fe55b5bd2bb)
Figure 1:

As seen in Fig. 1 control algorithms are run on Windows PC along the Qcar simulation to maximize response time and minimize transport delay.  

# Object detection and localization

Our system is capable of identifying stop signs and calculating the corresponding distance from the vehicle. Additionally, it can discern traffic lights, ascertain their status (red/green), and determine the distance from the vehicle. To enhance our global plan we detect the right side of the road as well as the middle yellow line. These data are then used to control the correct position of the car on the global path.  We use the OpenCV library for detection in the first stage of the competition. We trained a neural network for detection. We tested the utilization of the NN during the development of the solution for the first stage. However, we decided not to use it in the first stage. For the second stage, we are planning on deploying the NN, since we got familiar with it and we already created automated procedures for dataset creation and labelling as well as training.

# optimal solution
As we got the competition scripts that control the traffic lights we can deduce that the state of both traffic lights will be switched at the same time and every five seconds. With the knowledge of the maximal speed of the Qcar and the time required to stop at the stop signs, we can compute the theoretical optimal time to run the course. With validation from simulation, we can use the so-called "green wave" to go through the course. This means we do not have to stop at a traffic light and we are only obliged to stop for 3s at a stop sign. Our computed optimal time was about 12s including the needed 6s wait time at stop signs.



