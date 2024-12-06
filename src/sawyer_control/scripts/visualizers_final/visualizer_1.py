#!/usr/bin/env python

# this script runs the sawyer-spline visualizer with no intentional path error

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from intera_motion_interface import MotionTrajectory, MotionWaypoint, MotionWaypointOptions
from intera_interface import Limb
from intera_core_msgs.msg import EndpointState

import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from scipy.interpolate import CubicSpline

import numpy as np
import random

import pygame
import threading

 
'''IMPORTANT VARIABLES'''
# point limits for gui
x_screen_limit_left: int = 200
x_screen_limit_right: int = 1720
y_screen_limit_top: int = 200
y_screen_limit_bottom: int = 1000

# x-coords of points: starting at 200, in intervals of 380 pixels
x_coordinate_interval: int = 190
x_coordinate_interval: int = 380

# starting and ending pose of sawyer
start_pose = Pose(position=Point(x=0.6, y=-0.6, z=0.3),
                    orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))
end_pose = Pose(position=Point(x=0.6, y=0.2, z=0.3),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))


'''Visualizer Display'''
class SawyerVisualizer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('positionSubscriber', anonymous=True)

        # Initialize Pygame
        pygame.init()
        self.window = pygame.display.set_mode((1920, 1420))
        self.background_color = (2, 114, 212)
        pygame.display.set_caption("Sawyer Real-Time Display")

        # --------------------------
        # Spline generation
        # --------------------------
        self.ik_motion = IKMotionWaypoint()
        self.spline_controlpoints, self.spline_waypoints, self.display_waypoints = self.ik_motion.generate_spline_waypoints(start_pose, end_pose)

        x_coords = [p[0] for p in self.display_waypoints]  # Get x coordinates
        y_coords = [p[1] for p in self.display_waypoints]  # Get y coordinates

        # converting to pixel values for display spline
        x_coords = self.convert_to_pixels_x(x_coords, -0.6, 0.2, 200, 1720)
        y_coords = self.convert_to_pixels_y(y_coords, 0.1, 0.5, 200, 1000)

        # # draw the smooth line
        self.smooth_points = list(zip(x_coords, y_coords))  # packaging new curve points back into tuples

        # setting and printing real coordinate values in cm
        for point in self.display_waypoints:
            pygame.draw.circle(self.window, (255, 0, 0), point, 1, 0)

        # --------------------------
        # Real-time display entity setup
        # --------------------------
        # Rectangle properties
        self.rect_color = (9, 179, 54)
        self.rect_width = 50
        self.rect_height = 50

        # Set initial position
        self.x = 800
        self.y = 1000
        self.position_initialized = True

        # Scale factors to convert robot coordinates to screen coordinates
        self.scale_x = 1900  # pixels per meter
        self.scale_y = 1950  # pixels per meter

        # Threading lock for position updates
        self.lock = threading.Lock()

        # Start ROS subscriber
        self.subscriber = rospy.Subscriber("/robot/limb/right/endpoint_state",
                                           EndpointState,
                                           self.position_callback)

        # --------------------------
        # Flags and threads setup
        # --------------------------
        self.robot_motion_active = False  # flag to indicate if robot is moving
        self.motion_thread = None  # thread to handle robot motion

        self.spacebar_disabled = False # flag to make sure user only presses spacebar once (issue with pressing multiple times, trajectory is sent twice per press except initial press)

    def convert_to_pixels_x(self, meter_values, m_min, m_max, p_min, p_max):
        pixel_values = []
    
        for m in meter_values:
            # linear scale conversion from meters to pixels
            p = p_min + ((m - m_min) / (m_max - m_min)) * (p_max - p_min)

            pixel_values.append(p)
    
        return pixel_values

    def convert_to_pixels_y(self, meter_values, m_min, m_max, p_min, p_max):
        pixel_values = []
    
        for m in meter_values:
            # linear scale conversion from meters to pixels
            p = p_min + ((m - m_min) / (m_max - m_min)) * (p_max - p_min)

            if(m > 0.3):
                p = 600 - (p - 600) # making pixels higher on screen (lower pixel value) if above the midpoint threshold 0.3 m
            else:
                p = 600 + (600 - p) # making pixels lower on screen (higher pixel value) if below the midpoint threshold 0.3 m

            pixel_values.append(p)
    
        return pixel_values

    def position_callback(self, msg):
        with self.lock:
            # convert robot coordinates to screen coordinates
            screen_x = (msg.pose.position.y * self.scale_x) + 1325  # TODO Edit here to change scale the real time display
            screen_y = (-msg.pose.position.z * self.scale_y) + 1150  # TODO Edit here to change scale the real time display

            # clamp values to screen boundaries
            self.x = max(0, min(1920 - self.rect_width, screen_x))
            self.y = max(0, min(1080 - self.rect_height, screen_y))

    # thread to separate real time display and robot motion actions
    def start_robot_motion(self):
        if not self.robot_motion_active:
            self.robot_motion_active = True
            self.motion_thread = threading.Thread(target=self.ik_motion.execute_trajectory, args=(self.spline_waypoints,))
            self.motion_thread.start()

    def run(self):
        clock = pygame.time.Clock()
        running = True

        while running and not rospy.is_shutdown():
            # handle PyGame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    # start trajectory when spacebar is pressed
                    if event.key == pygame.K_SPACE and not self.robot_motion_active and not self.spacebar_disabled: #not trajectory_started:
                        self.spacebar_disabled = True
                        self.start_robot_motion()

            # check if motion thread has completed
            if self.motion_thread and not self.motion_thread.is_alive():
                self.robot_motion_active = False

            # clear screen with background color
            self.window.fill(self.background_color)

            for point in self.smooth_points:
                # draw a circle at each waypoint
                pygame.draw.circle(self.window, (255, 0, 0), (int(point[0]), int(point[1])), 5)  #radius of 5 pixels

            # draw rectangle at current position
            with self.lock:
                pygame.draw.rect(self.window,
                                 self.rect_color,
                                 (int(self.x), int(self.y),
                                  self.rect_width, self.rect_height))

            # update display
            pygame.display.flip()

            # control frame rate
            clock.tick(60)

        pygame.quit()
        rospy.signal_shutdown("PyGame window closed")



'''ROBOT MOTION'''
class IKMotionWaypoint:
    def __init__(self, limb="right"):
        # rospy.init_node('ik_motion_waypoint')
 
        # Limb and IK solver setup
        self._limb = Limb(limb)
        self.sawyerchain = self.load_chain("sawyer.urdf", "base", "right_hand")
        self.num_joints = self.sawyerchain.getNrOfJoints()
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.sawyerchain)
        self.ik_joint_positions = kdl.JntArray(self.num_joints)
        self.ik_joint_positions_prev = kdl.JntArray(self.num_joints)
        
        # Setup motion trajectory
        self.traj = MotionTrajectory(limb=self._limb)
        rospy.loginfo("IK and MotionWaypoint setup complete.")
 
    def load_chain(self, filename, base_link, end_effector_link):
        with open(filename, "r") as urdf_file:
            urdf_string = urdf_file.read()
        robot = URDF.from_xml_string(urdf_string)
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            raise RuntimeError("Failed to parse the URDF model!")
        return tree.getChain(base_link, end_effector_link)

    # TO CALCULATE IK
    def calculate_ik(self, pose):
        pos = pose.position
        orientation = kdl.Rotation.Quaternion(pose.orientation.x, pose.orientation.y,
                                              pose.orientation.z, pose.orientation.w)
        frame = kdl.Frame(orientation, kdl.Vector(pos.x, pos.y, pos.z))
        result = self.ik_solver.CartToJnt(self.ik_joint_positions_prev, frame, self.ik_joint_positions)
        if result < 0:
            rospy.logerr("IK solution not found")
            return None
        for i in range(self.num_joints):
            self.ik_joint_positions_prev[i] = self.ik_joint_positions[i]
        return [self.ik_joint_positions[i] for i in range(self.num_joints)]

    # TO GENERATE POINTS FOR THE SPLINE PATH
    def generate_spline_waypoints(self, start_pose, end_pose, num_points=10):
        # spline control points in Cartesian space
        control_points = np.array([
            [start_pose.position.x, start_pose.position.y, start_pose.position.z],
            [0.6, -0.4, random.uniform(0.1, 0.5)],  # middle control points for spline
            [0.6, -0.2, random.uniform(0.1, 0.5)],
            [0.6, 0.0, random.uniform(0.1, 0.5)],
            [end_pose.position.x, end_pose.position.y, end_pose.position.z]
        ])

        t = np.linspace(0, 1, len(control_points))
        t_spline = np.linspace(0, 1, num_points) # linspace used for the robot's motion (10 waypoints for spline creation)
        
        x_spline = CubicSpline(t, control_points[:, 0])(t_spline)
        y_spline = CubicSpline(t, control_points[:, 1])(t_spline)
        z_spline = CubicSpline(t, control_points[:, 2])(t_spline)

        # for DISPLAY spline -- need only z values that will be converted
        t_spline_display = np.linspace(0, 1, 1000)
        
        # x_spline_display = CubicSpline(t, control_points[:, 0])(t_spline_display)
        y_spline_display = CubicSpline(t, control_points[:, 1])(t_spline_display)
        z_spline_display = CubicSpline(t, control_points[:, 2])(t_spline_display)
        
        # Generate Pose waypoints along the spline
        self.waypoints = []
        for x, y, z in zip(x_spline, y_spline, z_spline):
            pose = Pose(
                position=Point(x=x, y=y, z=z),
                orientation=start_pose.orientation  # Use start orientation for simplicity
            )
            self.waypoints.append(pose)

        # Generate display waypoints as list of tuples
        display_waypoints = list(zip(y_spline_display, z_spline_display))
        
        return control_points, self.waypoints, display_waypoints

    # TO ADD WAYPOINT TO LIST OF POINTS WHEN EXECUTING TRAJECTORY
    def add_waypoint(self, pose, limb_name="right_hand"):
        joint_angles = self.calculate_ik(pose)
        if joint_angles is None:
            rospy.logerr("Skipping waypoint due to IK failure")
            return False
        wpt_opts = MotionWaypointOptions(
            max_linear_speed=0.3,
            max_linear_accel=0.3,
            max_rotational_speed=0.3,
            max_rotational_accel=0.3,
            max_joint_speed_ratio=0.3
        )
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
        waypoint.set_joint_angles(joint_angles, limb_name)
        self.traj.append_waypoint(waypoint.to_msg())
        rospy.loginfo("Waypoint added at: {}".format(pose.position))
        return True

    def execute_trajectory(self, spline_waypoints):
        
        for pose in spline_waypoints:
            self.add_waypoint(pose)

        result = self.traj.send_trajectory()
        if result is None:
            rospy.logerr("Trajectory FAILED to send")
        elif result.result:
            rospy.loginfo("Trajectory successfully executed!")
        else:
            rospy.logerr(f"Trajectory execution failed with error {result.errorId}")



if __name__ == '__main__':
    sawyerVisualizer = SawyerVisualizer()
    sawyerVisualizer.run()