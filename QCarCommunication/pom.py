import numpy as np
from scipy.interpolate import interp1d

points = np.array([[0,0,0],[0,1,0],[1,1,0],[1,2,0]])
x = points[:,0]
y = points[:,1]
psi = points[:,2]

distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
cumulative_distances = np.concatenate(([0], np.cumsum(distances)))
print(cumulative_distances)
print(cumulative_distances>1)
print(np.argmax(cumulative_distances>1))
        

        # Define the number of points for interpolation and generate new distances
new_distances = np.linspace(0, cumulative_distances[-1], int(cumulative_distances[-1]*10))
print(new_distances)

        # Perform cubic spline interpolation
 
interp_x = interp1d(cumulative_distances, x, kind='linear')
interp_y = interp1d(cumulative_distances, y, kind='linear')

        # Calculate interpolated points
x_new = interp_x(new_distances)
y_new = interp_y(new_distances)

points_new = np.column_stack((x_new, y_new))

print(points.shape)
print(points_new.shape)

print()
print(cumulative_distances)
print(x)
print()
print(points[:5])