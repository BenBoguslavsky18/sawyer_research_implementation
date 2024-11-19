#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from intera_interface import Limb
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def display_end_effector_movement():
    # Initialize ROS node
    rospy.init_node('matplotlib_end_effector_visualizer')

    # Define the target path as a set of 3D points (this can be customized)
    target_path_points = np.array([
        [0.5, 0.0, 0.2],
        [0.6, -0.1, 0.3],
        [0.7, -0.2, 0.4],
        [0.6, -0.3, 0.3],
        [0.5, -0.4, 0.2]
    ])

    # Initialize the Limb interface to get the end-effector position
    limb = Limb()

    # Set up Matplotlib for real-time 3D plotting
    plt.ion()  # Enable interactive mode for live plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the target path
    ax.plot(target_path_points[:, 0], target_path_points[:, 1], target_path_points[:, 2], label='Target Path', color='green')
    
    # Plot setup
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('Sawyer Robot End-Effector Movement')
    ax.legend()

    # Arrays to hold the end-effector trajectory
    trajectory_x, trajectory_y, trajectory_z = [], [], []

    rate = rospy.Rate(10)  # 10 Hz update rate
    while not rospy.is_shutdown():
        # Get the current end-effector pose
        pose = limb.endpoint_pose()
        x, y, z = pose['position'].x, pose['position'].y, pose['position'].z
        
        # Append the current position to the trajectory
        trajectory_x.append(x)
        trajectory_y.append(y)
        trajectory_z.append(z)
        
        # Clear and update the plot
        ax.cla()  # Clear the current plot
        ax.plot(target_path_points[:, 0], target_path_points[:, 1], target_path_points[:, 2], label='Target Path', color='green')
        ax.plot(trajectory_x, trajectory_y, trajectory_z, label='End-Effector Trajectory', color='red')
        ax.scatter(x, y, z, color='blue', s=50, label='Current Position')  # Blue dot for the real-time position

        # Reapply plot settings
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')
        ax.set_title('Sawyer Robot End-Effector Movement')
        ax.legend()

        plt.draw()
        plt.pause(0.01)  # Small pause for the plot to update

        rate.sleep()

if __name__ == '__main__':
    try:
        display_end_effector_movement()
    except rospy.ROSInterruptException:
        pass

