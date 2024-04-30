import numpy as np
import matplotlib.pyplot as plt
#from scipy.interpolate import CubicSpline, interp1d

class path:
    def __init__(self,x,y,psi,d):
        self.x = x
        self.y = y
        self.psi = psi
        self.d = d


class global_path:
    def __init__(self, np_file):
        self.name = np_file
        self.old_points = np.load(self.name)
        self.path = path(0,0,0,0)
        self.FF = None
        self.v_ref = None

    def get_global_path(self,no_of_points,to_plot,interp_type='linear'):
        if(self.old_points.shape[1]>10):
            self.old_points = self.old_points.T
        x = self.old_points[:,0]
        y = self.old_points[:,1]
        d = self.distance_calculation(x,y)
        psi = self.heading_calculation(x,y)
        self.path.x = x
        self.path.y = y
        self.path.psi = psi
        self.path.d = d
        if(self.old_points.shape[1]>=4):
            self.FF = self.old_points[:,3]
        if(self.old_points.shape[1]>=5):
            self.v_ref = self.old_points[:,4]    
        if(to_plot):
            self.plot_scenario()
    
    
    def heading_calculation(self,x,y):
        diff_x = np.diff(x)
        diff_y = np.diff(y)
        heading = np.arctan2(diff_y,diff_x)
        heading = np.append(heading, heading[-1])
        return heading

    def distance_calculation(self,x,y):
        distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
        cumulative_distances = np.concatenate(([0], np.cumsum(distances)))
        distance = cumulative_distances
        return distance
    
    def plot_scenario(self):
        # Plot XY
        plt.figure(1)  # Create or select figure 1
        #plt.plot(self.old_points[:,0],self.old_points[:,1], 'bo', label='Original Points')
        plt.plot(self.path.x, self.path.y, 'rx', label='Interpolated Trajectory')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Cubic Spline Interpolation')
        plt.legend()
        plt.axis('equal')
        plt.grid(True)

        # Plot of heading
        plt.figure(2)  # Create or select figure 1
        plt.plot(self.path.d,np.degrees(self.path.psi), 'r-', label='Heading')
        plt.xlabel('d')
        plt.ylabel('\psi')
        plt.title('Heading')
        plt.legend()
        plt.grid(True)

        # Show plot
        plt.show()
        return
    