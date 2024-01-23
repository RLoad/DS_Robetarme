import numpy as np

# Points initiaux du chemin sur la planche (dans le plan XY)
points_initial = np.array([[x1, y1],
                           [x2, y2],
                           [x3, y3],
                           [x4, y4]])

# Position et orientation initiales de la planche dans l'espace
initial_position = np.array([initial_x, initial_y, initial_z])
initial_orientation_quaternion = np.array([initial_qw, initial_qx, initial_qy, initial_qz])

# Nouvelle position et orientation de la planche dans l'espace
new_position = np.array([new_x, new_y, new_z])
new_orientation_quaternion = np.array([new_qw, new_qx, new_qy, new_qz])

# Calculer la transformation homogène pour le déplacement
translation_matrix = np.eye(4)
translation_matrix[:3, 3] = new_position - initial_position

rotation_matrix = np.eye(4)
rotation_matrix[:3, :3] = np.dot(
    np.dot(tf.transformations.quaternion_matrix(initial_orientation_quaternion)[:3, :3],
           tf.transformations.quaternion_matrix(new_orientation_quaternion)[:3, :3]),
    rotation_matrix[:3, :3]
)

transformation_matrix = np.dot(translation_matrix, rotation_matrix)

# Appliquer la transformation aux points du chemin
transformed_points = np.dot(transformation_matrix, np.vstack([points_initial.T, np.ones(points_initial.shape[0])]))

# Les nouveaux points sont dans transformed_points[:2, :]. Vous pouvez les utiliser pour le nouveau chemin.
