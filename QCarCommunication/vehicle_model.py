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




# # Example usage
# length = 2.5  # Length of the vehicle (meters)
# lf = length/2
# lr = length/2
# vehicle_model = KinematicVehicleModel(lr,lf,0,0,0)

# # Initial position and orientation
# x0 = 0
# y0 = 0
# theta0 = np.radians(90)  # Convert initial orientation to radians
# beta0 = 0
# velocity = 5  # m/s
# dt = 0.01  # Time step (seconds)

# # Given array of points
# no_of_points = 1000
# deltas = np.load('delta.npy')
# deltas = delta_interpolation(deltas,1000)
# deltas = np.deg2rad(deltas)

# # vehicle movement
# xC = np.zeros(no_of_points)
# yC = np.zeros(no_of_points)
# psiC = np.zeros(no_of_points)
# xC[0] = x0
# yC[0] = y0
# psiC[0] = theta0
# beta = beta0

# for i in range(no_of_points-1):
#     xC[i+1], yC[i+1], psiC[i+1], beta = vehicle_model.move(xC[i], yC[i], psiC[i], beta, velocity, deltas[i], dt)



# # Plot of heading
# plt.figure(1)  # Create or select figure 1
# plt.plot(xC,yC, 'r-', label='XY of car')
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('XY of car')
# plt.legend()
# plt.axis('equal')
# plt.grid(True)

# # Plot of delta
# plt.figure(2)  # Create or select figure 1
# plt.plot(np.linspace(0,dt*no_of_points,no_of_points),np.rad2deg(deltas), 'r-', label='\delta')
# plt.xlabel('t')
# plt.ylabel('\delta')
# plt.title('\delta')
# plt.legend()
# plt.grid(True)

# # Plot of psi
# plt.figure(3)  # Create or select figure 1
# plt.plot(np.linspace(0,dt*no_of_points,no_of_points),np.rad2deg(psiC), 'r-', label='\psi')
# plt.xlabel('t')
# plt.ylabel('\psi')
# plt.title('\psi')
# plt.legend()
# plt.grid(True)


# # Show plot
# plt.show()




