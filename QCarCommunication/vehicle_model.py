import numpy as np
import matplotlib.pyplot as plt


def delta_interpolation(delta,no_of_points):
    delta = np.interp(np.linspace(0, len(delta) - 1, no_of_points), np.arange(len(delta)), delta)
    return delta

class vehicle_position:
    def __init__(self,x,y,psi):
        self.x = x
        self.y = y
        self.psi = psi
        
class DeltaEstimator:
    def __init__(self,tau,a1,a2,a3):
        self.estimatedDelta = 0
        self.tau = tau
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        
    def calcDelta(self,delta,timeDiff):
        self.estimatedDelta = self.estimatedDelta + (timeDiff)*(-(1/self.tau)*self.estimatedDelta + (1/self.tau)*delta)
        self.estimatedDelta = np.sign(self.estimatedDelta)*(self.a1*abs(self.estimatedDelta)**3 + self.a2*abs(self.estimatedDelta)**2 + self.a3*abs(self.estimatedDelta))
    
    def getActualDelta(self):
        return self.estimatedDelta

class KinematicVehicleModel:
    def __init__(self, lr, lf, x0, y0, psi0):
        self.lr = lr  # Length of the vehicle
        self.lf = lf
        self.position = vehicle_position(x0,y0,psi0)

    def move(self, vc, delta, dt):
        """
        Move the vehicle based on its kinematic model.

        Parameters:
            x: float, current x position
            y: float, current y position
            theta: float, current orientation (radians)
            velocity: float, velocity of the vehicle (m/s)
            delta: float, steering angle (radians)
            dt: float, time step (seconds)

        Returns:
            New x, y, and theta after moving the vehicle.
        """
        # Calculate new position and orientation
        length = self.lr + self.lf
        beta = np.arctan2(self.lr * np.tan(delta), length)
        
        self.position.x += vc * np.cos(self.position.psi + beta) * dt
        self.position.y += vc * np.sin(self.position.psi + beta) * dt
        self.position.psi += ((vc * np.cos(beta)) / length) * np.tan(delta) * dt
        Xc = self.position.x
        Yc = self.position.y
        theta = self.position.psi

        return Xc, Yc, theta

