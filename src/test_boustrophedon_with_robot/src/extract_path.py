import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from mpl_toolkits.mplot3d import Axes3D

def quaternion_to_rotation_matrix(quaternion):
    # Convert quaternion to rotation matrix
    r = Rotation.from_quat(quaternion)
    return r.as_matrix()

def create_rectangle_points(x_obj, y_obj, quaternion, width=64, height=49):
    # Convert quaternion to rotation matrix
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)

    # Calculate local coordinates of rectangle corners
    local_points = np.array([
        [-width/2, -height/2, 0],
        [width/2, -height/2, 0],
        [width/2, height/2, 0],
        [-width/2, height/2, 0]
    ])

    # Transform to global coordinates
    global_points = np.dot(local_points, rotation_matrix.T) + np.array([x_obj, y_obj, 0])

    return global_points

def plot_3d_rectangle(global_points):
    # Plotting in 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Connect the rectangle corners in the order they were calculated
    corners = np.array([
        [global_points[0], global_points[1]],
        [global_points[1], global_points[2]],
        [global_points[2], global_points[3]],
        [global_points[3], global_points[0]]
    ])
    
    for corner in corners:
        ax.plot(corner[:, 0], corner[:, 1], corner[:, 2], 'ro-')

    ax.set_title('3D Rectangle')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    plt.show()

# Example usage with quaternion
x_obj = 50
y_obj = 30
quaternion = [1, 0, 0, 0.0]  # Example quaternion for 45 degrees rotation about Z-axis

rectangle_points = create_rectangle_points(x_obj, y_obj, quaternion)
plot_3d_rectangle(rectangle_points)
