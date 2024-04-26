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
        #x,y = self.path_interpolation(self.old_points[:,0],self.old_points[:,1],no_of_points,interp_type)
        if(self.old_points.shape[1]>5):
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
    
    # def path_interpolation(self,x,y,no_of_points,interp_type):
    #     # Calculate the cumulative distances between consecutive points
    #     distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    #     cumulative_distances = np.concatenate(([0], np.cumsum(distances)))
        

    #     # Define the number of points for interpolation and generate new distances
    #     new_distances = np.linspace(0, cumulative_distances[-1], no_of_points)

    #     # Perform cubic spline interpolation
    #     cs_x = CubicSpline(cumulative_distances, x)
    #     cs_y = CubicSpline(cumulative_distances, y)

    #     interp_x = interp1d(cumulative_distances, x, kind='linear')
    #     interp_y = interp1d(cumulative_distances, y, kind='linear')

    #     # Calculate interpolated points
    #     if(interp_type=='cubic'):
    #         x_new = cs_x(new_distances)
    #         y_new = cs_y(new_distances)
    #     else:
    #         x_new = interp_x(new_distances)
    #         y_new = interp_y(new_distances)
    #     return x_new,y_new
    
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
    


# file_path = 'waypoints.txt'  # Replace 'your_file.txt' with the actual file path

# data = np.loadtxt(file_path, delimiter=',')
# print(data.shape)

# data2=np.load('Lturn.npy')
# print(data2.shape)
# np.save('QuanserTraj.npy', data)




# Original set of points
#points = np.array([[0,0],[1,0],[3,0],[4,0],[5,0],[5,1],[5,2],[5,3],[5,5]])
#np.save('Lturn.npy', points)



# Plot XY
# plt.figure(1)  # Create or select figure 1
# plt.plot(points[:,0],points[:,1], 'bo', label='Original Points')
# plt.plot(x, y, 'rx', label='Interpolated Trajectory')
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Cubic Spline Interpolation')
# plt.legend()
# plt.axis('equal')
# plt.grid(True)

# Plot of heading
# plt.figure(2)  # Create or select figure 1
# plt.plot(d,np.degrees(psi), 'r-', label='Heading')
# plt.xlabel('d')
# plt.ylabel('\psi')
# plt.title('Heading')
# plt.legend()
# plt.grid(True)


# Show plot
# plt.show()