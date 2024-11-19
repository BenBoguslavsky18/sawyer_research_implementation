#!/usr/bin/env python
import pygame
from random import randint
import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Empty
import intera_interface
from intera_interface import CHECK_VERSION


import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel


'''
Initializing Window
'''
pygame.init()
window = pygame.display.set_mode((1920,1420)) #setting window size
background_color = (2, 114, 212)
window.fill(background_color)
pygame.display.set_caption("Sawyer's Path")

# TUPLE LIST FOR STORING X AND Y COORDINATES TO SEND TO SAWYER
path_coordinates = []

#Creating points limits for gui
x_screen_limit_left = 200
x_screen_limit_right = 1720
y_screen_limit_top = 200
y_screen_limit_bottom = 1000

#X-coords of points: Starting at 200, in intervals of 380 pixels
x_coordinate_interval = 190
x_coordinate_interval = 380


'''
Curve and point generation
'''
class CoordinateConverter:
    def __init__(self):
        # Screen dimensions in pixels
        self.x_screen_limit_left = x_screen_limit_left
        self.x_screen_limit_right = x_screen_limit_right
        self.y_screen_limit_top = y_screen_limit_top
        self.y_screen_limit_bottom = y_screen_limit_bottom
        
        # Desired robot workspace dimensions in centimeters
        self.x_robot_range = 75  # 75cm total horizontal range
        self.y_robot_range = 50  # 50cm total vertical range
        
        # Calculate conversion factors
        self.x_pixel_range = self.x_screen_limit_right - self.x_screen_limit_left
        self.y_pixel_range = self.y_screen_limit_bottom - self.y_screen_limit_top
        
        self.x_scale = self.x_robot_range / self.x_pixel_range
        self.y_scale = self.y_robot_range / self.y_pixel_range

    def pixels_to_cm(self, points):
        """Convert pixel coordinates to centimeter coordinates."""
        cm_points = []
        
        for px, py in points:
            # Normalize coordinates to 0-1 range
            x_normalized = (px - self.x_screen_limit_left) / self.x_pixel_range
            y_normalized = (py - self.y_screen_limit_top) / self.y_pixel_range
            
            # Convert to centimeters, ranging from 0 to max
            x_cm = (x_normalized * self.x_robot_range) / 100  # Will range from 0 to 75cm (left to right)
            y_cm = - (y_normalized * self.y_robot_range - 25) / 100 # Will range from 0 to +=25cm
            
            cm_points.append((x_cm, y_cm))
            
        return cm_points

def create_spline():
    converter = CoordinateConverter()
    
    #list of 5 points (tuples) used to create curve path
    points = [(200 + i * x_coordinate_interval, randint(y_screen_limit_top, y_screen_limit_bottom) if 0 < i < 4 else 600) for i in range(5)] #setting first and last point to middle height

    #drawing coordinates and points
    for index, point in enumerate(points):

        #drawing coordinate points
        pygame.draw.circle(window, (255, 255, 255), [points[index][0], points[index][1]], 10, 0) #0 line width makes a filled in circle

        #connecting dots with linear lines
        pygame.draw.lines(window, (255, 255, 255), False, points, 2) # False means the line is not closed, 2 is the width

    #Spline interpolation to smooth the line
    x_coords, y_coords = zip(*points) #separates x and y values of points into separate objects
    tck, u = splprep([x_coords, y_coords], s=2) #tck is 'tuple of spline representation', nessecary data to define the curve. 'u' is just parameters used in interpolation
    u_fine = np.linspace(0, 1, 1000) #'u_fine' is parameters for the new points of the smooth curves. 'np.linespace' generates 1000 evenly spaced values between 0 and 1
    x_fine, y_fine = splev(u_fine, tck) #new set of curve points

    #draw the smooth line
    smooth_points = list(zip(x_fine, y_fine)) #packaging new curve points back into tuples
    
    global path_coordinates
    path_coordinates = converter.pixels_to_cm(smooth_points)

    #setting and printing real coordinate values in cm
    pygame.draw.lines(window, (255, 0, 0), False, smooth_points, 4)  # False means the line is not closed, 4 is the width


        # Plot with Matplotlib to show centimeter values
    # plt.figure(figsize=(8, 6))
    # x_cm, y_cm = zip(*path_coordinates)
    # plt.plot(x_cm, y_cm, color='green', label='Robot Path')
    # plt.title('Robot Path in Centimeters')
    # plt.xlabel('X (cm)')
    # plt.ylabel('Y (cm)')
    # plt.grid(True)
    # plt.legend()
    
        # Set axis limits to show full range
    # plt.xlim(0, 75)
    # plt.ylim(0, 50)
    # plt.axis('equal')  # Make the plot scale equally
    # plt.show()


'''
Motion Teleop
'''
# starting position
starting_x = 0.1
starting_y = -0.7

#rate
rate_frequency = 1000.0

teleop = None #responsible for movement

class Teleop(object):
    # initialization method
    def __init__(self, limb = "right"):
        # -------------------------------------
        # Saywer setup
        # -------------------------------------
        # control parameters
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout
 
        # create right limb instance
        self._limb = intera_interface.Limb(limb)
        self._arm_joints = self._limb.joint_names()
 
        # initialize parameters
        self._start_angles = dict()
 
        # create cuff disable publisher / disabling cuff
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)
 
        # verify robot is enabled
        print("Getting robot state...")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot...\n")
        self._rs.enable()
 
        # Setup IK for follower arm (Sawyer)
        # chain for follower arm
        self.sawyerchain = self.load_chain("sawyer.urdf", "base", "right_hand")
        
        # Create a KDL joint array for follower arm
        self.num_joints = self.sawyerchain.getNrOfJoints()
        # Create the IK solver
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.sawyerchain)
 
        # Create an empty joint array to store the IK solution
        self.ik_joint_positions = kdl.JntArray(self.num_joints)
        self.ik_joint_positions_prev = kdl.JntArray(self.num_joints)
 
        print("Running. Ctrl-c to quit")
 
 
    #extract the kinematic chain between base link and end effector
    def load_chain(self, filename, base_link, end_effector_link):
        # Load the URDF from a file
        with open(filename, "r") as urdf_file:
            urdf_string = urdf_file.read()
            
        # Load the URDF from the string
        robot = URDF.from_xml_string(urdf_string)
        ok, tree = treeFromUrdfModel(robot)
 
        # Ensure the URDF was successfully parsed
        if not ok:
            raise RuntimeError("Failed to parse the URDF model!")
            
        # Extract the kinematic chain from base to end-effector
        return tree.getChain(base_link, end_effector_link)
 
 
    #do IK for given coordinates
    def _solve_ik(self, x, y, z, orientation): 
        # Perform IK on Sawyer
        # Convert position and orientation to PyKDL.Vector and PyKDL.Rotation
        position_vector = kdl.Vector(x, y, z)
        orientation_rotation = kdl.Rotation(orientation[0, 0], orientation[0, 1], orientation[0, 2],
                                            orientation[1, 0], orientation[1, 1], orientation[1, 2],
                                            orientation[2, 0], orientation[2, 1], orientation[2, 2])
    
 
        # Set the desired end-effector frame for IK
        target_frame = kdl.Frame(orientation_rotation, position_vector)
        # Perform inverse kinematics
        result = self.ik_solver.CartToJnt(self.ik_joint_positions_prev, target_frame, self.ik_joint_positions) #guess-start location, target, stored result
 
        for i in range(self.num_joints):
            self.ik_joint_positions_prev[i] = self.ik_joint_positions[i]
 
        return result
 
 
    #attempt to set robot joint positions based on result of IK
    def _map_poses(self, x, y, z, orientation):
        result = self._solve_ik(x, y, z, orientation)
        if result >= 0:
            # record current angles/velocities
            cur_pos = self._limb.joint_angles()
            i = 0
            for j in self._arm_joints:
                cur_pos[j] = self.ik_joint_positions[i]
                i = i+1
            self._limb.set_joint_positions(cur_pos)
        else:
            print("[ERROR] Could not solve IK")
 

    #moves the limb to neutral location
    def move_to_neutral(self):
        self._limb.move_to_neutral()
        cur_pos = self._limb.joint_angles()
        i = 0
        for j in self._arm_joints:
            self.ik_joint_positions_prev[i] = cur_pos[j]
            i = i+1
 
 
    def run_teleop(self, x_coordinate, y_coordinate):
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()
 
        # set control rate
        control_rate = rospy.Rate(rate_frequency)
 
        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and return to Position Control Mode
        self._limb.set_command_timeout((1.0 / rate_frequency) * self._missed_cmds)
 
        print("Done, ready to move.\n")

        # read the current end effector pose
        cur_ee_pose = self._limb.endpoint_pose()
        # x = -0.3
        # y = -0.7
        x = x_coordinate
        y = y_coordinate
        z = 0.2 # KEEP CONSTANT
        orientation = kdl.Rotation.Quaternion(cur_ee_pose["orientation"].x, cur_ee_pose["orientation"].y, cur_ee_pose["orientation"].z, cur_ee_pose["orientation"].w)
 
        # loop at specified rate commanding new joint torques
        # while not rospy.is_shutdown():
        while not (x - 0.001) < cur_ee_pose["position"].x < (x + 0.001) and not (y - 0.001) < cur_ee_pose["position"].y < (y + 0.001) and not (z - 0.001) < cur_ee_pose["position"].z < (z + 0.001):
            if not self._rs.state().enabled:
                rospy.logerr("Running the robot")
                break
            self._map_poses(x, y, z, orientation)
            cur_ee_pose = self._limb.endpoint_pose()
            print(cur_ee_pose["position"].x)
            control_rate.sleep()
 
 
    #exit cleanly
    def clean_shutdown(self):
        print("\nExiting example...")
        self._limb.exit_control_mode()


'''
Startup function to return to start
'''
def startup():
    # Starting node connection to ROS
    teleop = Teleop()
    # register shutdown callback
    # rospy.on_shutdown(teleop.clean_shutdown)
    teleop.move_to_neutral()

    # [Yue] You need to call the run_teleop to run the control loop, otherwise it is doing nothing
    # teleop.run_teleop(-0.3, -0.7)
    teleop.run_teleop(starting_x, starting_y)


def main():
    print("Initializing node... ")
    rospy.init_node("sawyer_arm_teleop")
    startup()

    # try:
    create_spline()
    pygame.display.update()
    # except rospy.ROSInterruptException:
    #     pass

    #---main loop--- TODO NEEDS TO BE UPDATED FOR REAL TIME GRAPHICS EVENTUALLY
    while True:

        #preprocessing of inputs
        eventList = pygame.event.get()
        for event in eventList:
            if event.type == pygame.QUIT:
                pygame.quit()


        #rendering
        pygame.display.update()      

if __name__ == "__main__":
    main()