import rospy
from sensor_msgs.msg import JointState

class JointStateSubscriber:
    def __init__(self):
        rospy.init_node('joint_state_subscriber', anonymous=True)
        self.joint_states = None

        # Define the joint names
        self.joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # Subscribe to the JointState topic
        rospy.Subscriber('/ur5/joint_states', JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        # Store the joint states
        self.joint_states = msg

        # Extract positions, velocities, and efforts
        positions = msg.position
        velocities = msg.velocity
        efforts = msg.effort

        # Store position separately
        position_dict = dict(zip(self.joint_names, positions))
        
        # Print or use the stored positions as needed
        print("Joint Positions:")
        for joint_name, joint_position in position_dict.items():
            print(f"{joint_name}: {joint_position}")

if __name__ == '__main__':
    joint_state_subscriber = JointStateSubscriber()

    # Spin to keep the script running
    rospy.spin()
