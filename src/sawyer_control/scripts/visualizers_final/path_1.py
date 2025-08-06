#!/usr/bin/env python

#  This script runs the sawyer-spline visualizer with no intentional path error

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

import pygame
import threading


'''IMPORTANT VARIABLES'''
x_screen_limit_left: int = 200
x_screen_limit_right: int = 1720
y_screen_limit_top: int = 200
y_screen_limit_bottom: int = 1000

start_pose = Pose(position=Point(x=0.75, y=-0.3, z=0.3),
                  orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))
end_pose = Pose(position=Point(x=0.75, y=0.5, z=0.3),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))


'''Visualizer Display'''
class SawyerVisualizer:
    def __init__(self):
        rospy.init_node('positionSubscriber', anonymous=True)

        pygame.init()
        self.window = pygame.display.set_mode((1920, 1420))
        self.background_color = (2, 114, 212)
        pygame.display.set_caption("Sawyer Real-Time Display")

        self.ik_motion = IKMotionWaypoint()
        self.spline_controlpoints, self.spline_waypoints, self.display_waypoints = self.ik_motion.generate_spline_waypoints(start_pose, end_pose)

        x_coords = [p[0] for p in self.display_waypoints]
        y_coords = [p[1] for p in self.display_waypoints]
        x_coords = self.convert_to_pixels_x(x_coords, -0.3, 0.5, 200, 1720)
        y_coords = self.convert_to_pixels_y(y_coords, 0.1, 0.5, 200, 1000)
        self.smooth_points = list(zip(x_coords, y_coords))
        self.trail_points = [] #list of points to store the path moves by the robot when going along the trajectory

        self.icon_color = (9, 179, 54) # the real-time display position icon
        self.icon_radius = 30
        self.x = 800
        self.y = 1000
        self.position_initialized = True
        self.scale_x = 1900
        self.scale_y = 1950
        self.lock = threading.Lock()

        self.subscriber = rospy.Subscriber("/robot/limb/right/endpoint_state",
                                           EndpointState,
                                           self.position_callback)

        self.robot_motion_active = False
        self.robot_ready_to_move = False
        self.motion_thread = None
        self.spacebar_disabled = False

    def convert_to_pixels_x(self, meter_values, m_min, m_max, p_min, p_max):
        return [p_min + ((m - m_min) / (m_max - m_min)) * (p_max - p_min) for m in meter_values]

    def convert_to_pixels_y(self, meter_values, m_min, m_max, p_min, p_max):
        pixel_values = []
        for m in meter_values:
            p = p_min + ((m - m_min) / (m_max - m_min)) * (p_max - p_min)
            if m > 0.3:
                p = 600 - (p - 600)
            else:
                p = 600 + (600 - p)
            pixel_values.append(p)
        return pixel_values

    def position_callback(self, msg):
        with self.lock:
            screen_x = (msg.pose.position.y * self.scale_x) + 770 #original 1340
            screen_y = (-msg.pose.position.z * self.scale_y) + 1180
            self.x = max(0, min(1920 - self.icon_radius, screen_x)) # the position on the screen
            self.y = max(0, min(1080 - self.icon_radius, screen_y)) # the position on the screen

    def move_to_start_position(self):
        rospy.loginfo("Moving to start position...")
        if not self.robot_ready_to_move:
            # Create a threading Event to signal when the move is complete
            self.start_position_event = threading.Event()
            
            def move_and_signal():
                self.ik_motion.move_to_pose(start_pose)
                # Set the event to signal the move is complete
                self.start_position_event.set()
            
            # Start the thread for moving to start position
            self.start_position_thread = threading.Thread(target=move_and_signal)
            self.start_position_thread.start()
            
            # Start another thread to wait for completion without blocking the main thread
            def wait_for_move_completion():
                self.start_position_event.wait()
                # Use rospy.is_shutdown() to check if ROS is still running
                if not rospy.is_shutdown():
                    self.robot_ready_to_move = True
                    self.spacebar_disabled = False  # Re-enable spacebar
                    rospy.loginfo("Reached start position.")
            
            self.wait_thread = threading.Thread(target=wait_for_move_completion)
            self.wait_thread.start()

    def record_realtime_position(self):
        self.trail_points.append((int(self.x), int(self.y)))

    def start_robot_motion(self):
        if self.robot_ready_to_move and not self.robot_motion_active:
            self.robot_motion_active = True
            self.motion_thread = threading.Thread(target=self.ik_motion.execute_trajectory, args=(self.spline_waypoints,))
            self.motion_thread.start()

    def run(self):
        clock = pygame.time.Clock()
        running = True

        guide_line_position_offset: int = 0

        while running and not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        if not self.robot_ready_to_move and not self.spacebar_disabled:  # First press
                            self.spacebar_disabled = True
                            self.move_to_start_position()
                        elif not self.robot_motion_active:  # Second press
                            self.start_robot_motion()


            self.window.fill(self.background_color)
            for point in self.smooth_points:
                pygame.draw.circle(self.window, (255, 0, 0), ((int(point[0]), point[1])), 5)
            
            if(self.robot_motion_active and self.robot_ready_to_move): #draw real time trail when following trajectory
                self.record_realtime_position()
                for point in self.trail_points:
                    pygame.draw.circle(self.window, (0, 0, 0), (point[0], point[1]), 5)

                #draw pace line here
                if(self.smooth_points[0][0] + guide_line_position_offset < self.smooth_points[-1][0]):
                    pygame.draw.line(self.window, (235, 168, 52),  (self.smooth_points[0][0] + guide_line_position_offset, self.smooth_points[0][1] - 1000), (self.smooth_points[0][0] + guide_line_position_offset, self.smooth_points[0][1] + 1000), 5)
                    guide_line_position_offset += 0.75
                else:
                    pygame.draw.line(self.window, (235, 168, 52),  (self.smooth_points[-1][0], self.smooth_points[0][1] - 1000), (self.smooth_points[-1][0], self.smooth_points[-1][1] + 1000), 5)
            else:
                pygame.draw.line(self.window, (235, 168, 52),  (self.smooth_points[0][0], self.smooth_points[0][1] - 1000), (self.smooth_points[0][0], self.smooth_points[0][1] + 1000), 5)

            with self.lock:
                pygame.draw.circle(self.window, self.icon_color, (int(self.x), int(self.y)), self.icon_radius)#, self.rect_height))
            
            pygame.display.flip()
            clock.tick(60)

        pygame.quit()
        rospy.signal_shutdown("PyGame window closed")


'''ROBOT MOTION'''
class IKMotionWaypoint:
    def __init__(self, limb="right"):
        self._limb = Limb(limb)
        self.sawyerchain = self.load_chain("sawyer.urdf", "base", "right_hand")
        self.num_joints = self.sawyerchain.getNrOfJoints()
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.sawyerchain)
        self.ik_joint_positions = kdl.JntArray(self.num_joints)
        self.ik_joint_positions_prev = kdl.JntArray(self.num_joints)
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

    def move_to_pose(self, pose, limb_name="right_hand"):
        joint_angles = self.calculate_ik(pose)
        if joint_angles is None:
            rospy.logerr("Failed to move to pose: IK solution not found")
            return
        wpt_opts = MotionWaypointOptions(
            max_linear_speed=0.1,
            max_linear_accel=0.1,
            max_rotational_speed=0.1,
            max_rotational_accel=0.1,
            max_joint_speed_ratio=0.1
        )
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
        waypoint.set_joint_angles(joint_angles, limb_name)
        self.traj.clear_waypoints()
        self.traj.append_waypoint(waypoint.to_msg())
        result = self.traj.send_trajectory()
        if result is None:
            rospy.logerr("Failed to move to pose: Trajectory failed to send")
        elif result.result:
            rospy.loginfo("Successfully moved to pose!")
        else:
            rospy.logerr(f"Failed to move to pose with error {result.errorId}")

    def generate_spline_waypoints(self, start_pose, end_pose, num_points=40):
        control_points = np.array([
            [start_pose.position.x, start_pose.position.y, start_pose.position.z],
            [0.75, -0.1, 0.4],
            [0.75, 0.1, 0.2],
            [0.75, 0.3, 0.4],
            [end_pose.position.x, end_pose.position.y, end_pose.position.z]
        ])

        t = np.linspace(0, 1, len(control_points))
        t_spline = np.linspace(0, 1, num_points)
        x_spline = CubicSpline(t, control_points[:, 0])(t_spline)
        y_spline = CubicSpline(t, control_points[:, 1])(t_spline)
        z_spline = CubicSpline(t, control_points[:, 2])(t_spline)

        t_spline_display = np.linspace(0, 1, 1000)
        y_spline_display = CubicSpline(t, control_points[:, 1])(t_spline_display)
        z_spline_display = CubicSpline(t, control_points[:, 2])(t_spline_display)

        self.waypoints = [Pose(position=Point(x=x, y=y, z=z), orientation=start_pose.orientation)
                          for x, y, z in zip(x_spline, y_spline, z_spline)]

        display_waypoints = list(zip(y_spline_display, z_spline_display))
        return control_points, self.waypoints, display_waypoints
    
    def add_waypoint(self, pose, limb_name="right_hand"):
        """
        Adds a waypoint to the trajectory based on the given pose.
        """
        joint_angles = self.calculate_ik(pose)
        if joint_angles is None:
            rospy.logerr("Skipping waypoint due to IK failure")
            return False
        wpt_opts = MotionWaypointOptions(
            max_linear_speed=0.1,
            max_linear_accel=0.1,
            max_rotational_speed=0.1,
            max_rotational_accel=0.1,
            max_joint_speed_ratio=0.1
        )
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
        waypoint.set_joint_angles(joint_angles, limb_name)
        self.traj.append_waypoint(waypoint.to_msg())
        rospy.loginfo("Waypoint added at: {}".format(pose.position))
        return True


    def execute_trajectory(self, spline_waypoints):
        """
        Executes a trajectory based on the given spline waypoints.
        """
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
