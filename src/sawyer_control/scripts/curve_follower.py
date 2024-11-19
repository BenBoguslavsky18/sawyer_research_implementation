#! /usr/bin/env python
"""
This script generates a spline and display for sawyer to follow-- for Amr and Ben's research
"""
# -----------------------------------------------------------
#ros imports
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Empty

#intera/sawyer imports
import intera_interface
from intera_interface import CHECK_VERSION

#other libraries for IK, Robot trees 
import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel

#libraries for motion gui display
import pygame
from random import randint
import numpy as np
from scipy.interpolate import splprep, splev
# -----------------------------------------------------------
import matplotlib.pyplot as plt

"""
PATH DISPLAY
"""
#-------------------
#Initializing Window
#-------------------
pygame.init()
window = pygame.display.set_mode((1920,1420)) #setting window size
background_color = (2, 114, 212)
window.fill(background_color)
pygame.display.set_caption("Sawyer's Path")

# TUPLE LIST FOR STORING X AND Y COORDINATES TO SEND TO SAWYER
path_coordinates = []

#-------------------
#Creating points limits for gui
#-------------------
x_screen_limit_left = 200
x_screen_limit_right = 1720
y_screen_limit_top = 200
y_screen_limit_bottom = 1000

#X-coords of points: Starting at 200, in intervals of 380 pixels
x_coordinate_interval = 190
x_coordinate_interval = 380

def create_spline():
    #list of 5 points (tuples) used to create curve path
    points = [(200 + i * x_coordinate_interval, randint(y_screen_limit_top, y_screen_limit_bottom) if 0 < i < 4 else 600) for i in range(5)] #setting first and last point to middle height

    #drawing coordinates and points
    for index, point in enumerate(points):

        #drawing coordinate points
        pygame.draw.circle(window, (255, 255, 255), [points[index][0], points[index][1]], 10, 0) #0 line width makes a filled in circle

        #connecting dots with linear lines
        pygame.draw.lines(window, (255, 255, 255), False, points, 2) # False means the line is not closed, 2 is the width

        #-------------------
        #Spline interpolation to smooth the line
        #-------------------
        x_coords, y_coords = zip(*points) #separates x and y values of points into separate objects
        tck, u = splprep([x_coords, y_coords], s=2) #tck is 'tuple of spline representation', nessecary data to define the curve. 'u' is just parameters used in interpolation
        u_fine = np.linspace(0, 1, 10) #'u_fine' is parameters for the new points of the smooth curves. 'np.linespace' generates 1000 evenly spaced values between 0 and 1
        x_fine, y_fine = splev(u_fine, tck) #new set of curve points

        # Draw the smooth line
        smooth_points = list(zip(x_fine, y_fine)) #packaging new curve points back into tuples
        
        global path_coordinates
        path_coordinates = [(x/2000 - 0.1, -y/1500 + 0.4) for x, y in smooth_points] #list of coords to send to sawyer



        # 4. Plotting with Matplotlib
        plt.figure(figsize=(8, 6))

        # Plot the original points (as circles)
        # plt.scatter(x_coords, y_coords, color='red', label='Original Points')

        # Plot the smooth spline curve
        # plt.plot(x_fine, y_fine, color='blue', label='Smooth Spline Curve')

        plt.plot(path_coordinates, color='green', label='Curve')

        # Optionally plot the line connecting original points (linear interpolation)
        # plt.plot(x_coords, y_coords, 'gray', linestyle='--', label='Linear Interpolation')

        # Labeling the axes and the plot
        plt.title('Spline Curve Interpolation')
        plt.xlabel('X')
        plt.ylabel('Y')

        # Show legend
        plt.legend()

        # Display the plot
        plt.grid(True)
        plt.show()







        #setting and printing real coordinate values in cm
        pygame.draw.lines(window, (255, 0, 0), False, smooth_points, 4)  # False means the line is not closed, 4 is the width

# PUBLISHER ~~topic: coordinates~~
def coordinatePublisher():
    rospy.init_node('coordinatePublisher')
    coordinate_publisher = rospy.Publisher('coordinates', Pose2D, queue_size=10)
    rate = rospy.Rate(1000) #1000 Hz

    for coord in path_coordinates:
        pose = Pose2D()
        pose.x = coord[0]
        pose.y = coord[1]
        pose.theta = 0.0

        rospy.loginfo(f"Publishing: ROBOT_X: {pose.x}, ROBOT_Y: {pose.y}")
        coordinate_publisher.publish(pose)
        rate.sleep()


def createAndPublishSpline():
    try:
        create_spline()
        pygame.display.update()
        coordinatePublisher()
        pygame.display.update()
    except rospy.ROSInterruptException:
        print("Error running the publisher")
        pass

    #---main loop--- TODO NEEDS TO BE UPDATED FOR REAL TIME GRAPHICS EVENTUALLY
    # while True:

    #     # preprocessing of inputs
    #     eventList = pygame.event.get()
    #     for event in eventList:
    #         if event.type == pygame.QUIT:
    #             pygame.quit()


    #     # rendering
    #     pygame.display.update()  



"""
CARTESIAN MOTION
"""
# ---------------------
# Important global vars
# ---------------------
starting_x = -0.3
starting_y = -0.7


class Teleop(object):
 
    # initialization method
    def __init__(self, limb = "right"):
        # -------------------------------------
        # Saywer setup
        # -------------------------------------
        # control parameters
        self._rate = 1000.0  # Hz
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
        control_rate = rospy.Rate(self._rate)
 
        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and return to Position Control Mode
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)
 
        print("Done, ready to move.\n")

        # read the current end effector pose
        cur_ee_pose = self._limb.endpoint_pose()
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
 
 
def startup():
    # Starting node connection to ROS
    print("Initializing node... ")
    # rospy.init_node("sawyer_arm_teleop")
    teleop = Teleop()
    # register shutdown callback
    rospy.on_shutdown(teleop.clean_shutdown)
    teleop.move_to_neutral()

    teleop.run_teleop(starting_x, starting_y)


# SUBSCRIBER ~~topic: coordinates~~ add to start coordinates 
def coordinateSubscriber():
    rospy.init_node('sawyerControl')
    rospy.Subscriber('coordinates', Pose2D, moveToNewCooridinates)
    rospy.spin()

def moveToNewCooridinates(data):
    x_displacement = data.x
    y_displacement = data.y
    teleop.run_teleop(starting_x + x_displacement, starting_y+y_displacement)

 

# ----
# MAIN
# ----
if __name__ == "__main__":
    
    # First create the spline and display it
    create_spline()
    pygame.display.update()

    # Initialize the robot and subscriber first
    print("Initializing node... ")
    rospy.init_node('sawyerControl')
    
    # Create and setup the teleop instance
    global teleop
    teleop = Teleop()
    rospy.on_shutdown(teleop.clean_shutdown)

    # Move to starting position
    teleop.move_to_neutral()
    teleop.run_teleop(starting_x, starting_y)

    # Setup the subscriber
    coordinate_sub = rospy.Subscriber('coordinates', Pose2D, moveToNewCooridinates)
    
    # Setup the publisher
    coordinate_pub = rospy.Publisher('coordinates', Pose2D, queue_size=10)
    rate = rospy.Rate(50)  # Slowed down rate for better coordination

    # Now publish coordinates while the subscriber is active
    for coord in path_coordinates:
        if rospy.is_shutdown():
            break
            
        pose = Pose2D()
        pose.x = coord[0]
        pose.y = coord[1]
        pose.theta = 0.0

        rospy.loginfo(f"Publishing: ROBOT_X: {pose.x}, ROBOT_Y: {pose.y}")
        coordinate_pub.publish(pose)
        rate.sleep()
    
    # Keep the node running to process any remaining coordinates
    rospy.spin()