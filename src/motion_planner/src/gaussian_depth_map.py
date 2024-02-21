import rospy
from geometry_msgs.msg import Pose
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal

# Assuming you have a callback function to handle incoming Pose messages
def pose_callback(msg):
    x = msg.position.x
    y = msg.position.y
    z = msg.position.z
    
    # Create a 2D Gaussian distribution based on X and Y coordinates
    mean = [x, y]
    covariance_matrix = np.eye(2)  # Identity matrix as a simple example
    
    # Create a grid of X and Y values
    x_grid, y_grid = np.meshgrid(np.linspace(x - 1, x + 1, 100), np.linspace(y - 1, y + 1, 100))
    
    # Evaluate the Gaussian distribution at each point in the grid
    pos = np.dstack((x_grid, y_grid))
    rv = multivariate_normal(mean, covariance_matrix)
    z = rv.pdf(pos)
    
    # Create a contour plot with the Gaussian distribution
    plt.contourf(x_grid, y_grid, z, cmap='viridis')
    plt.colorbar(label='Probability Density')
    
    # Scatter plot the actual end effector position on top
    plt.scatter(x, y, c='red', marker='o', label='End Effector Pose')
    
    # Set labels and title
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Height Map with Gaussian Distribution')
    
    # Show the plot
    plt.legend()
    plt.show()

# Assuming you have a ROS node that subscribes to the /ur5/ee_info/Pose topic
rospy.init_node('height_map_node')
rospy.Subscriber('/ur5/ee_info/Pose', Pose, pose_callback)

# Spin to keep the script running
rospy.spin()
