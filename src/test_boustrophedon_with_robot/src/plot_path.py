import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def quaternion_to_rotation_matrix(q):
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    rotation_matrix = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])
    return rotation_matrix

def rotate_vertex(vertex, rotation_matrix, position):
    rotated_vertex = np.dot(rotation_matrix, (vertex - position)) + position
    return rotated_vertex

# Replace these values with your object's position and quaternion
object_position = np.array([1.0, 2.0, 3.0])
object_quaternion = np.array([0.6, 0.36, 0.36, -0.6])

# Step 1: Convert quaternion to rotation matrix
rotation_matrix = quaternion_to_rotation_matrix(object_quaternion)

# Step 2: Apply rotation to each vertex of the object (dummy vertices for illustration)
vertices = np.array([[1.0, 0.0, 0.0],
                     [0.0, 1.0, 0.0],
                     [0.0, 0.0, 1.0]])

rotated_vertices = np.array([rotate_vertex(vertex, rotation_matrix, object_position) for vertex in vertices])

# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Original vertices
ax.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], label='Original Vertices', marker='o')

# Rotated vertices
ax.scatter(rotated_vertices[:, 0], rotated_vertices[:, 1], rotated_vertices[:, 2], label='Rotated Vertices', marker='^')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Rotation of 3D Object')

ax.legend()
plt.show()
