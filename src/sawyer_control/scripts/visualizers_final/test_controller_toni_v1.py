#!/usr/bin/env python

#  Sawyer path visualizer with low impedance direct-to-endpoint motion

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from intera_motion_interface import MotionTrajectory, MotionWaypoint, MotionWaypointOptions, InteractionOptions
from intera_interface import Limb
from intera_core_msgs.msg import EndpointState, InteractionControlCommand
from std_msgs.msg import Empty

import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from scipy.interpolate import CubicSpline

import numpy as np
import pygame
import threading
import time


'''IMPORTANT VARIABLES'''
x_screen_limit_left: int = 200
x_screen_limit_right: int = 1720
y_screen_limit_top: int = 200
y_screen_limit_bottom: int = 1000

start_pose = Pose(position=Point(x=0.75, y=-0.3, z=0.3),
                  orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))
end_pose = Pose(position=Point(x=0.75, y=0.5, z=0.3),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))


def create_interaction_msg(stiffness):
    """
    Creates and configures an InteractionControlCommand message with the given stiffness.
    """
    interaction_options = InteractionOptions()
    interaction_options.set_interaction_control_active(True)
    interaction_options.set_K_impedance(stiffness)  # Cartesian stiffness
    interaction_options.set_max_impedance([False, False, False, False, False, False])  # Not maximizing stiffness
    interaction_options.set_interaction_control_mode([1, 1, 1, 1, 1, 1])  # Impedance mode for all axes
    interaction_options.set_in_endpoint_frame(False)  # Base frame as reference
    interaction_options.set_force_command([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # No external force commands
    interaction_options.set_K_nullspace([5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0])  # Nullspace stiffness
    interaction_options.set_disable_damping_in_force_control(False)  # Enable damping
    interaction_options.set_disable_reference_resetting(False)  # Allow smooth transitions

    return interaction_options.to_msg()


def create_position_control_msg():
    """
    Creates a message to switch the robot back to position control mode.
    """
    msg = InteractionControlCommand()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base"
    msg.interaction_control_active = False  # Disable interaction control
    return msg


'''Visualizer Display'''
class SawyerVisualizer:
    def __init__(self):
        rospy.init_node('impedance_spline_tracer', anonymous=True)

        pygame.init()
        self.window = pygame.display.set_mode((1920, 1420))
        self.background_color = (2, 114, 212)
        pygame.display.set_caption("Sawyer Low Impedance Spline Tracer")

        self.ik_motion = IKMotionWaypoint()
        self.spline_controlpoints, self.spline_waypoints, self.display_waypoints = self.ik_motion.generate_spline_waypoints(start_pose, end_pose)

        x_coords = [p[0] for p in self.display_waypoints]
        y_coords = [p[1] for p in self.display_waypoints]
        x_coords = self.convert_to_pixels_x(x_coords, -0.3, 0.5, 200, 1720)
        y_coords = self.convert_to_pixels_y(y_coords, 0.1, 0.5, 200, 1000)
        self.smooth_points = list(zip(x_coords, y_coords))
        self.trail_points = []

        self.icon_color = (9, 179, 54)
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
        self.trajectory_complete = False

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
            screen_x = (msg.pose.position.y * self.scale_x) + 770
            screen_y = (-msg.pose.position.z * self.scale_y) + 1180
            self.x = max(0, min(1920 - self.icon_radius, screen_x))
            self.y = max(0, min(1080 - self.icon_radius, screen_y))

    def move_to_start_position(self):
        rospy.loginfo("Moving to start position...")
        if not self.robot_ready_to_move:
            self.start_position_event = threading.Event()
            
            def move_and_signal():
                self.ik_motion.move_to_pose(start_pose)
                self.start_position_event.set()
            
            self.start_position_thread = threading.Thread(target=move_and_signal)
            self.start_position_thread.start()
            
            def wait_for_move_completion():
                self.start_position_event.wait()
                if not rospy.is_shutdown():
                    self.robot_ready_to_move = True
                    self.spacebar_disabled = False
                    rospy.loginfo("Reached start position. Ready for low impedance motion.")
            
            self.wait_thread = threading.Thread(target=wait_for_move_completion)
            self.wait_thread.start()

    def record_realtime_position(self):
        self.trail_points.append((int(self.x), int(self.y)))

    def start_robot_motion(self):
        if self.robot_ready_to_move and not self.robot_motion_active:
            self.robot_motion_active = True
            rospy.loginfo("Starting low impedance motion to final point...")
            
            def motion_wrapper():
                self.ik_motion.execute_low_impedance_trajectory(self.spline_waypoints)
                # Mark trajectory as complete and stop the game
                self.trajectory_complete = True
                rospy.loginfo("Trajectory complete. Stopping visualization...")
            
            self.motion_thread = threading.Thread(target=motion_wrapper)
            self.motion_thread.start()

    def run(self):
        clock = pygame.time.Clock()
        running = True

        while running and not rospy.is_shutdown() and not self.trajectory_complete:
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
            
            # Draw spline path as guide
            for point in self.smooth_points:
                pygame.draw.circle(self.window, (255, 0, 0), ((int(point[0]), point[1])), 5)
            
            # Draw real-time trail when following trajectory
            if self.robot_motion_active and self.robot_ready_to_move:
                self.record_realtime_position()
                for point in self.trail_points:
                    pygame.draw.circle(self.window, (0, 0, 0), (point[0], point[1]), 5)

            with self.lock:
                pygame.draw.circle(self.window, self.icon_color, (int(self.x), int(self.y)), self.icon_radius)
            
            pygame.display.flip()
            clock.tick(60)

        pygame.quit()
        rospy.signal_shutdown("PyGame window closed")


'''ROBOT MOTION WITH IMPEDANCE CONTROL'''
class IKMotionWaypoint:
    def __init__(self, limb="right"):
        self._limb = Limb(limb)
        self.sawyerchain = self.load_chain("sawyer.urdf", "base", "right_hand")
        self.num_joints = self.sawyerchain.getNrOfJoints()
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.sawyerchain)
        self.ik_joint_positions = kdl.JntArray(self.num_joints)
        self.ik_joint_positions_prev = kdl.JntArray(self.num_joints)
        self.traj = MotionTrajectory(limb=self._limb)
        
        # Impedance control setup
        self.impedance_pub = rospy.Publisher('/robot/limb/right/interaction_control_command',
                                           InteractionControlCommand, queue_size=10)
        self.collision_suppress_pub = rospy.Publisher('/robot/limb/right/suppress_collision_avoidance',
                                                    Empty, queue_size=10)
        
        # Ultra-low stiffness for maximum manual guidance
        self.low_stiffness = [0.5, 0.5, 0.5, 2.0, 2.0, 2.0]  # Much lower than original
        
        rospy.loginfo("IK and Low Impedance Motion setup complete.")

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
            [0.75, -0.1, 0.2],
            [0.75, 0.1, 0.4],
            [0.75, 0.3, 0.2],
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

    def create_ultra_low_impedance_msg(self):
        """
        Creates impedance message with ultra-low stiffness for maximum manual guidance.
        """
        interaction_options = InteractionOptions()
        interaction_options.set_interaction_control_active(True)
        interaction_options.set_K_impedance(self.low_stiffness)
        interaction_options.set_max_impedance([False, False, False, False, False, False])
        interaction_options.set_interaction_control_mode([1, 1, 1, 1, 1, 1])
        interaction_options.set_in_endpoint_frame(False)
        interaction_options.set_force_command([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        interaction_options.set_K_nullspace([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])  # Lower nullspace
        interaction_options.set_disable_damping_in_force_control(False)
        interaction_options.set_disable_reference_resetting(False)
        return interaction_options.to_msg()

    def enable_low_impedance(self):
        """Enable low impedance mode for manual guidance"""
        rospy.loginfo("Enabling ultra-low impedance mode...")
        
        # Suppress collision avoidance
        self.collision_suppress_pub.publish(Empty())
        
        # Set very low stiffness
        low_impedance_msg = self.create_ultra_low_impedance_msg()
        self.impedance_pub.publish(low_impedance_msg)
        
        rospy.loginfo("Ultra-low impedance mode enabled. Robot can be easily guided manually.")

    def disable_impedance_control(self):
        """Disable impedance control and return to position mode"""
        rospy.loginfo("Disabling impedance control...")
        position_control_msg = create_position_control_msg()
        for _ in range(5):  # Publish multiple times to ensure mode switch
            self.impedance_pub.publish(position_control_msg)
            rospy.sleep(0.2)
        rospy.loginfo("Returned to position control mode.")

    def execute_low_impedance_trajectory(self, spline_waypoints):
        """
        Move through waypoints gradually with very low impedance for better manual guidance.
        """
        try:
            # Enable low impedance mode first
            self.enable_low_impedance()
            rospy.sleep(1.0)  # Allow impedance mode to activate
            
            # Use more waypoints for smoother motion (every 3rd waypoint)
            step_size = max(1, len(spline_waypoints) // 12)  # Create ~12 intermediate goals
            selected_waypoints = spline_waypoints[::step_size]
            if spline_waypoints[-1] not in selected_waypoints:
                selected_waypoints.append(spline_waypoints[-1])  # Ensure we reach the end
            
            rospy.loginfo(f"Moving through {len(selected_waypoints)} intermediate waypoints...")
            
            # Start impedance maintenance thread
            impedance_active = threading.Event()
            impedance_active.set()
            
            def maintain_low_impedance():
                rate = rospy.Rate(20)  # Higher frequency for better control
                while impedance_active.is_set() and not rospy.is_shutdown():
                    self.collision_suppress_pub.publish(Empty())
                    low_impedance_msg = self.create_ultra_low_impedance_msg()
                    self.impedance_pub.publish(low_impedance_msg)
                    rate.sleep()
            
            impedance_thread = threading.Thread(target=maintain_low_impedance)
            impedance_thread.daemon = True
            impedance_thread.start()
            
            # Move through each waypoint with pauses for manual guidance
            for i, waypoint_pose in enumerate(selected_waypoints):
                if rospy.is_shutdown():
                    break
                    
                rospy.loginfo(f"Moving to waypoint {i+1}/{len(selected_waypoints)}")
                
                # Set very gentle motion parameters
                wpt_opts = MotionWaypointOptions(
                    max_linear_speed=0.005,      # Very slow
                    max_linear_accel=0.003,      # Gentle acceleration
                    max_rotational_speed=0.01,
                    max_rotational_accel=0.01,
                    max_joint_speed_ratio=0.03   # Slow joint motion
                )
                
                # Calculate IK for current waypoint
                joint_angles = self.calculate_ik(waypoint_pose)
                if joint_angles is None:
                    rospy.logwarn(f"Failed to calculate IK for waypoint {i+1}, skipping...")
                    continue
                
                # Create waypoint
                waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
                waypoint.set_joint_angles(joint_angles, "right_hand")
                
                # Execute motion to this waypoint
                self.traj.clear_waypoints()
                self.traj.append_waypoint(waypoint.to_msg())
                
                result = self.traj.send_trajectory(timeout=30.0)  # Longer timeout for slow motion
                
                if result is None or not result.result:
                    rospy.logwarn(f"Failed to reach waypoint {i+1}")
                
                # Brief pause between waypoints to allow manual adjustment
                rospy.sleep(0.8)
            
            rospy.loginfo("Completed all waypoints!")
            
            # Stop impedance maintenance
            impedance_active.clear()
            
        except Exception as e:
            rospy.logerr(f"Error during low impedance trajectory: {e}")
        
        finally:
            # Always disable impedance control when done
            rospy.sleep(1.0)
            self.disable_impedance_control()


if __name__ == '__main__':
    try:
        sawyerVisualizer = SawyerVisualizer()
        sawyerVisualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt detected. Exiting...")