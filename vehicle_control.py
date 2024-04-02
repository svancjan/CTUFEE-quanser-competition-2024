# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports

"""
vehicle_control.py

Skills acivity code for vehicle control lab guide.
Students will implement a vehicle speed and steering controller.
Please review Lab Guide - vehicle control PDF
"""
import os
import signal
import numpy as np
from threading import Thread
import time
import cv2
import csv
import pyqtgraph as pg

from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.utilities.scope import MultiScope
from pal.utilities.math import wrap_to_pi
from hal.products.qcar import QCarEKF
from hal.products.mats import SDCSRoadMap
import pal.resources.images as images

import matplotlib.pyplot as plt

# Car constants
WHEEL_RADIUS = 0.0342 # front/rear wheel radius in m
ENCODER_COUNTS_PER_REV = 720.0 # counts per revolution
WHEEL_BASE = 0.256 # front to rear wheel distance in m
WHEEL_TRACK = 0.17 # left to right wheel distance in m
PIN_TO_SPUR_RATIO = (13.0*19.0) / (70.0*37.0)
    # (diff_pinion*pinion) / (spur*diff_spur)
CPS_TO_MPS = (1/(ENCODER_COUNTS_PER_REV*4) # motor-speed unit conversion
    * PIN_TO_SPUR_RATIO * 2*np.pi * WHEEL_RADIUS)


#================ Experiment Configuration ================
# ===== Timing Parameters
# - tf: experiment duration in seconds.
# - startDelay: delay to give filters time to settle in seconds.
# - controllerUpdateRate: control update rate in Hz. Shouldn't exceed 500
tf = 20

startDelay = 1
controllerUpdateRate = 100

current = [] #battery current array
gyrox = [] #gyro x,y,z roational speed
gyroy = []
gyroz = []
accerx = [] #gyro x,y,z acceleration
accery = []
accerz = []
contSpeed = []
motorSpeed = [] #motor speeed
timeLog = [] #timevector log
x_log  = []
y_log  = [] #log state estimate
y_gps_log = []
th_log = []
t_move = []
gazu = []
#contSpeed = np.array([]) #front wheel contact speed


# ===== Speed Controller Parameters
# - v_ref: desired velocity in m/s
# - K_p: proportional gain for speed controller
# - K_i: integral gain for speed controller
v_ref = 5
K_p = 1000
K_i = 500

# ===== Steering Controller Parameters
# - enableSteeringControl: whether or not to enable steering control
# - K_stanley: K gain for stanley controller
# - nodeSequence: list of nodes from roadmap. Used for trajectory generation.
enableSteeringControl = True
K_stanley = 5
nodeSequence = [0, 20, 0]

#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Initial setup
if enableSteeringControl:
    roadmap = SDCSRoadMap(leftHandTraffic=False)
    waypointSequence = roadmap.generate_path(nodeSequence)
    initialPose = roadmap.get_node_pose(nodeSequence[0]).squeeze()
else:
    initialPose = [0, 0, 0]

if not IS_PHYSICAL_QCAR:
    import qlabs_setup
    qlabs_setup.setup(
        initialPosition=[initialPose[0], initialPose[1], 0],
        initialOrientation=[0, 0, initialPose[2]]
    )

# Used to enable safe keyboard triggered shutdown
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)
#endregion

class SpeedFeedForwardController:
    def __init__(self, k=1):
        self.L1 = 5.34
        self.L2 = 15.2
        self.S1 = 6.84
        self.S2 = 11.17
        self.Path = 15.5
        self.x = 0
        self.v = 0
        self.v_ref = 0
        self.stop_done1 = False
        self.stop_done2 = False
        self.u_last = 0
        self.maxThrottle = 1
        self.time_slept = 0



    def update(self, v, dt):
        u = self.u_last
        a = -1/0.24 * v + 9.4/0.24 * u
        #v = a*dt
        #self.v += v
        self.x += v*dt
        safety = 0.1
        print("self.x: ", self.x)

        # First run to Stop 1
        if self.x < (self.S1):
            self.v_ref = 4
        else: 
            if self.stop_done1 == False:
                self.v_ref = 0
                # wait for 3 seconds
                if(self.time_slept < 3):
                    self.time_slept += dt
                else:
                    self.v_ref = 4
                    self.stop_done1 = True
                    self.time_slept = 0
                
            
            if (self.x < self.S2) and (self.stop_done1 == True):
                self.v_ref = 4
            elif(self.stop_done1 == True):
                if self.stop_done2 == False:
                    self.v_ref = 0
                    print("waiting at stop 2", self.time_slept)
                    # wait 3 seconds
                    if(self.time_slept < 3) and (self.stop_done2 == False):
                        self.time_slept += dt
                    else:  
                        self.v_ref = 4
                        self.stop_done2 = True
                        self.time_slept = 0

                if (self.x < self.Path) and (self.stop_done2 == True):
                    self.v_ref = 4
                if (self.x > self.Path) and (self.stop_done2 == True):
                    self.v_ref = 0
                    print("I am at the finish line")



        print("v_ref: ", self.v_ref)
        if 1 < self.v_ref - 0.1:
            out = 1  # Turn ON the GAZU
        elif 1 > self.v_ref + 0.1:
            out = 0  # Turn OFF the GAZU
        else:
            out = 0

        self.u_last = out
        return np.clip(
            out,
            -self.maxThrottle, #min
            self.maxThrottle   #max
        )





class SpeedController:

    def __init__(self, kp=0, ki=0):
        self.maxThrottle = 1

        self.kp = kp
        self.ki = ki

        self.ei = 0


    # ==============  SECTION A -  Speed Control  ====================
    def update(self, v, v_ref, dt):
        
        #e = v_ref - v
        #if abs(e) - 0.1 > 0:
        #    out = 1
        #else:
        #    out = 0
        if v < v_ref - 0.1:
            out = 1  # Turn ON the GAZU
        elif v > v_ref + 0.1:
            out = -1  # Turn OFF the GAZU
        else:
            out = 0

        return np.clip(
            out,
            -self.maxThrottle, #min
            self.maxThrottle   #max
        )
        #return np.clip(
        #    self.kp*e + self.ki*self.ei,
        #    -self.maxThrottle, #min
        #    self.maxThrottle   #max
        #)
        
        return 0

class SteeringController:

    def __init__(self, waypoints, k=1, cyclic=True):
        self.maxSteeringAngle = np.pi/6

        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0

        self.k = k
        self.cyclic = cyclic

        self.p_ref = (0, 0)
        self.th_ref = 0

    # ==============  SECTION B -  Steering Control  ====================
    def update(self, p, th, speed):
        wp_1 = self.wp[:, np.mod(self.wpi, self.N-1)]
        wp_2 = self.wp[:, np.mod(self.wpi+1, self.N-1)]
        
        v = wp_2 - wp_1
        v_mag = np.linalg.norm(v)
        try:
            v_uv = v / v_mag
        except ZeroDivisionError:
            return 0

        tangent = np.arctan2(v_uv[1], v_uv[0])

        s = np.dot(p-wp_1, v_uv)

        if s >= v_mag:
            if  self.cyclic or self.wpi < self.N-2:
                self.wpi += 1

        ep = wp_1 + v_uv*s
        ct = ep - p
        dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)

        ect = np.linalg.norm(ct) * np.sign(dir)
        psi = wrap_to_pi(tangent-th)

        self.p_ref = ep
        self.th_ref = tangent

        return np.clip(
            wrap_to_pi(psi + np.arctan2(self.k*ect, speed)),
            -self.maxSteeringAngle,
            self.maxSteeringAngle)
        
        return 0


def controlLoop():
    #region controlLoop setup
    global KILL_THREAD
    u = 0 #gazu
    delta = 0 #steering
    v_ref = 5
    # used to limit data sampling to 10hz
    countMax = controllerUpdateRate /100
    count = 0
    lastenc = None
    tenclast = None
    #endregion

    #region Controller initialization
    speedController = SpeedFeedForwardController()

    """
    speedController = SpeedController(
        kp=K_p,
        ki=K_i
    )
    """

    if enableSteeringControl:
        steeringController = SteeringController(
            waypoints=waypointSequence,
            k=K_stanley
        )
    #endregion
    x_total = 0
    #region QCar interface setup
    qcar = QCar(readMode=1, frequency=controllerUpdateRate)
    if enableSteeringControl:
        ekf = QCarEKF(x_0=initialPose)
        gps = QCarGPS(initialPose=initialPose)
    else:
        gps = memoryview(b'')
    #endregion
    with qcar, gps:
        t0 = time.time()
        t=0
        t_start = 0
        setttle = 0
        while (t < tf+startDelay) and (not KILL_THREAD):
            #region : Loop timing update
            tp = t
            t = time.time() - t0
            dt = t-tp
            #endregion
            if t % 5 > 4.99:
                print(v_ref)
                if v_ref > 0:
                    v_ref = 0
                    t_start = 0
                else:
                    v_ref = 5
                    t_start = time.time()
                    setttle = 0

            #region : Read from sensors and update state estimates
            qcar.read()
            if enableSteeringControl:
                if gps.readGPS():
                    y_gps = np.array([
                        gps.position[0],
                        gps.position[1],
                        gps.orientation[2]
                    ])
                    ekf.update(
                        [qcar.motorTach, delta],
                        dt,
                        y_gps,
                        qcar.gyroscope[2],
                    )
                else:
                    ekf.update(
                        [qcar.motorTach, delta],
                        dt,
                        None,
                        qcar.gyroscope[2],
                    )

                x = ekf.x_hat[0,0]
                y = ekf.x_hat[1,0]
                th = ekf.x_hat[2,0]
                # log state ----------------------------
                x_total += x
                y_gps_log.append(gps.position[1])
                th_log.append(th)
                t_move.append(t)
                # --------------------------------------
                p = ( np.array([x, y])
                    + np.array([np.cos(th), np.sin(th)]) * 0.2)
                
            v = qcar.motorTach
            if (v > 0.93*3.6) and v_ref == 5 and setttle == 0:
                print(t_start-time.time())
                setttle = 1
            #endregion

            #region : Update controllers and write to car
            if t < startDelay:
                u = 0
                delta = 0
            else:
                #region : Speed controller update¨

                #if  (1 <= t < 2.9) or (5.9 <= t < 7.1) or (10.1 <= t < 11.3):
                #    v_ref = 4
                #else:
                #    v_ref = 0
                if  (t > 0.1):
                    u = speedController.update(v, dt)
                else:
                    u = 0

                
                #endregion

                #region : Steering controller update
                if enableSteeringControl:
                    delta = steeringController.update(p, th, v)
                else:
                    delta = 0
                #endregion
            
            qcar.write(u, 0)

            #endregion

            #region : Update Scopes
            count += 1
            if count >= countMax and t > startDelay:
                t_plot = t - startDelay


                if lastenc is None:  
                    print(t_plot)
                    tenclast = time.time()            
                    contSpeed.append(qcar.motorEncoder[0]-qcar.motorEncoder[0])
                    lastenc = qcar.motorEncoder[0]
                    #np.concatenate(contSpeed, qcar.motorEncoder, axis=0)
                    #contSpeed.append(qcar.motorEncoder - qcar.motorEncoder)
                else:
                    temp = time.time()
                    if abs(temp - tenclast) >1:
                        contSpeed.append(None)
                        print("Error sample")
                    else:
                        cmpSpeed = ((qcar.motorEncoder[0] - lastenc)/(temp - tenclast))*CPS_TO_MPS
                        contSpeed.append(cmpSpeed)

                    #print((qcar.motorEncoder[0] - lastenc)/(temp - tenclast))
                    lastenc = qcar.motorEncoder[0]
                    tenclast = temp
                gazu.append(u)
                current.append(qcar.motorCurrent)          
                gyrox.append(qcar.gyroscope[0])        
                gyroy.append(qcar.gyroscope[1])        
                gyroz.append(qcar.gyroscope[2])
                accerx.append(qcar.accelerometer[0])
                accery.append(qcar.accelerometer[1])
                accerz.append(qcar.accelerometer[2])
                motorSpeed.append(qcar.motorTach)
                timeLog.append(t_plot)
                y_log.append(p[1])
                x_log.append(p[0])

                # Speed control scope
                speedScope.axes[0].sample(t_plot, [v, v_ref])
                speedScope.axes[1].sample(t_plot, [v_ref-v])
                speedScope.axes[2].sample(t_plot, [u])

                # Steering control scope
                if enableSteeringControl:
                    steeringScope.axes[4].sample(t_plot, [[p[0],p[1]]])

                    p[0] = ekf.x_hat[0,0]
                    p[1] = ekf.x_hat[1,0]

                    x_ref = steeringController.p_ref[0]
                    y_ref = steeringController.p_ref[1]
                    th_ref = steeringController.th_ref

                    x_ref = gps.position[0]
                    y_ref = gps.position[1]
                    th_ref = gps.orientation[2]

                    steeringScope.axes[0].sample(t_plot, [p[0], x_ref])
                    steeringScope.axes[1].sample(t_plot, [p[1], y_ref])
                    steeringScope.axes[2].sample(t_plot, [th, th_ref])
                    steeringScope.axes[3].sample(t_plot, [delta])


                    arrow.setPos(p[0], p[1])
                    arrow.setStyle(angle=180-th*180/np.pi)

                count = 0
            #endregion
            continue

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Setup and run experiment
if __name__ == '__main__':

    #region : Setup scopes
    if IS_PHYSICAL_QCAR:
        fps = 10
    else:
        fps = 30
    # Scope for monitoring speed controller
    speedScope = MultiScope(
        rows=3,
        cols=1,
        title='Vehicle Speed Control',
        fps=fps
    )
    speedScope.addAxis(
        row=0,
        col=0,
        timeWindow=tf,
        yLabel='Vehicle Speed [m/s]',
        yLim=(0, 10)
    )
    speedScope.axes[0].attachSignal(name='v_meas', width=2)
    speedScope.axes[0].attachSignal(name='v_ref')

    speedScope.addAxis(
        row=1,
        col=0,
        timeWindow=tf,
        yLabel='Speed Error [m/s]',
        yLim=(-0.5, 0.5)
    )
    speedScope.axes[1].attachSignal()

    speedScope.addAxis(
        row=2,
        col=0,
        timeWindow=tf,
        xLabel='Time [s]',
        yLabel='Throttle Command [%]',
        yLim=(-0.3, 1)
    )
    speedScope.axes[2].attachSignal()

    # Scope for monitoring steering controller
    if enableSteeringControl:
        steeringScope = MultiScope(
            rows=4,
            cols=2,
            title='Vehicle Steering Control',
            fps=fps
        )

        steeringScope.addAxis(
            row=0,
            col=0,
            timeWindow=tf,
            yLabel='x Position [m]',
            yLim=(-17.5, 1.5)
        )
        steeringScope.axes[0].attachSignal(name='x_meas')
        steeringScope.axes[0].attachSignal(name='x_ref')

        steeringScope.addAxis(
            row=1,
            col=0,
            timeWindow=tf,
            yLabel='y Position [m]',
            yLim=(-20, 1)
        )
        steeringScope.axes[1].attachSignal(name='y_meas')
        steeringScope.axes[1].attachSignal(name='y_ref')

        steeringScope.addAxis(
            row=2,
            col=0,
            timeWindow=tf,
            yLabel='Heading Angle [rad]',
            yLim=(-3.5, 3.5)
        )
        steeringScope.axes[2].attachSignal(name='th_meas')
        steeringScope.axes[2].attachSignal(name='th_ref')

        steeringScope.addAxis(
            row=3,
            col=0,
            timeWindow=tf,
            yLabel='Steering Angle [rad]',
            yLim=(-0.6, 0.6)
        )
        steeringScope.axes[3].attachSignal()
        steeringScope.axes[3].xLabel = 'Time [s]'

        steeringScope.addXYAxis(
            row=0,
            col=1,
            rowSpan=4,
            xLabel='x Position [m]',
            yLabel='y Position [m]',
            xLim=(-2.5, 2.5),
            yLim=(-1, 5)
        )

        steeringScope.axes[4].attachImage(
            scale=(-0.002035, 0.002035),
            offset=(1125,2365),
            rotation=180,
            levels=(0, 255)
        )
        #steeringScope.axes[4].images[0].setImage(image=im)

        referencePath = pg.PlotDataItem(
            pen={'color': (85,168,104), 'width': 2},
            name='Reference'
        )
        steeringScope.axes[4].plot.addItem(referencePath)
        referencePath.setData(waypointSequence[0, :],waypointSequence[1, :])

        steeringScope.axes[4].attachSignal(name='Estimated', width=2)

        arrow = pg.ArrowItem(
            angle=180,
            tipAngle=60,
            headLen=10,
            tailLen=10,
            tailWidth=5,
            pen={'color': 'w', 'fillColor': [196,78,82], 'width': 1},
            brush=[196,78,82]
        )
        arrow.setPos(initialPose[0], initialPose[1])
        steeringScope.axes[4].plot.addItem(arrow)
    #endregion

    #region : Setup control thread, then run experiment
    controlThread = Thread(target=controlLoop)
    controlThread.start()

    try:
        while controlThread.is_alive() and (not KILL_THREAD):
            MultiScope.refreshAll()
            time.sleep(0.01)
    finally:
        KILL_THREAD = True

    print("PLOT THE LOGGGS")
    print(len(timeLog), )
    # Create subplots
    fig, axs = plt.subplots(3, 2, figsize=(12, 8))

    # Plot battery current
    axs[0, 0].plot(timeLog, gazu, label='Throttle')
    axs[0, 0].set_title('Throttle request')
    axs[0, 0].set_xlabel('Time')
    axs[0, 0].set_ylabel('Current')

    # Plot gyro data
    axs[0, 1].plot(timeLog, gyrox, label='Gyro Rotational Speed x')
    axs[0, 1].plot(timeLog, gyroy, label='Gyro Rotational Speed y')
    axs[0, 1].plot(timeLog, gyroz, label='Gyro Rotational Speed z')
    axs[0, 1].set_title('Gyro Rotational Speed')
    axs[0, 1].set_xlabel('Time')
    axs[0, 1].set_ylabel('Speed')

    # Plot acceleration
    axs[1, 0].plot(timeLog, accerx, label='Gyro Acceleration x')
    axs[1, 0].plot(timeLog, accery, label='Gyro Acceleration y')
    axs[1, 0].plot(timeLog, accerz, label='Gyro Acceleration z')
    axs[1, 0].set_title('Gyro Acceleration')
    axs[1, 0].set_xlabel('Time')
    axs[1, 0].set_ylabel('Acceleration')
    axs[1, 0].legend()

    # Plot motor speed
    axs[1, 1].plot(timeLog, motorSpeed, label='Motor Speed')
    axs[1, 1].set_title('Motor Speed')
    axs[1, 1].set_xlabel('Time')
    axs[1, 1].set_ylabel('Speed')

    axs[2, 0].plot(t_move, th_log, label='naticení')
    axs[2, 0].set_title('Slip Ratio')
    axs[2, 0].set_xlabel('Time')
    axs[2, 0].set_ylabel('Slip ratio [-]')
    axs[2, 0].legend()

    # Plot lambda
    axs[2, 1].plot(timeLog, x_log, label='X')
    axs[2, 1].plot(timeLog, y_log, label='Y')
    axs[2, 1].set_title('Slip Ratio')
    axs[2, 1].set_xlabel('Time')
    axs[2, 1].set_ylabel('Slip ratio [-]')
    axs[2, 1].legend()

    # Add legend
    for ax in axs.flat:
        ax.legend()

    # Adjust layout
    plt.tight_layout()

    # Show the plots
    plt.show()

    assert len(timeLog) == len(motorSpeed), "Both lists must have the same length."

    # Open the file in write mode ('w')
    with open('output.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        print("Saving to file")

        # Write the lists to the file as two columns
        for item1, item2, item3 in zip(timeLog, motorSpeed, gazu):
            writer.writerow([item1, item2, item3])    

        #endregion
    if not IS_PHYSICAL_QCAR:
        qlabs_setup.terminate()

    input('Experiment complete. Press any key to exit...')
#endregion