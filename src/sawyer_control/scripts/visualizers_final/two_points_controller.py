#!/usr/bin/env python

#  Sawyer path visualizer with low impedance multi-trajectory motion

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
        pygame.display.set_caption("Sawyer Multi-Trajectory Low Impedance Tracer")
        
        # Initialize font for timer display
        pygame.font.init()
        self.font = pygame.font.Font(None, 72)  # Large font for timer
        self.small_font = pygame.font.Font(None, 36)  # Smaller font for instructions

        self.ik_motion = IKMotionWaypoint()
        self.spline_controlpoints, self.spline_waypoints, self.display_waypoints = self.ik_motion.generate_spline_waypoints(start_pose, end_pose)

        x_coords = [p[0] for p in self.display_waypoints]
        y_coords = [p[1] for p in self.display_waypoints]
        x_coords = self.convert_to_pixels_x(x_coords, -0.3, 0.5, 200, 1720)
        y_coords = self.convert_to_pixels_y(y_coords, 0.1, 0.5, 200, 1000)
        self.smooth_points = list(zip(x_coords, y_coords))
        self.trail_points = []
        
        # Calculate final target position in pixels
        final_y = end_pose.position.y
        final_z = end_pose.position.z
        target_x_pixel = self.convert_to_pixels_x([final_y], -0.3, 0.5, 200, 1720)[0]
        target_y_pixel = self.convert_to_pixels_y([final_z], 0.1, 0.5, 200, 1000)[0]
        self.final_target_pixel = (int(target_x_pixel), int(target_y_pixel))

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
        
        # Multi-trajectory variables
        self.current_waypoint_index = 0
        self.trajectory_paused = False
        
        # Timer variables
        self.timer_started = False
        self.start_time = None
        self.elapsed_time = 0.0
        
        # Success detection variables
        self.success_radius = 50  # pixels - radius for success detection
        self.min_distance_to_target = float('inf')  # Track closest approach
        self.success_achieved = False
        
        # Waypoint tracking for visualization
        self.completed_waypoints = []
        self.current_target_pixel = None

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

    def get_current_target_pixel(self):
        """Get the pixel coordinates of the current target waypoint"""
        if self.current_waypoint_index < len(self.spline_waypoints):
            current_target = self.spline_waypoints[self.current_waypoint_index]
            target_x_pixel = self.convert_to_pixels_x([current_target.position.y], -0.3, 0.5, 200, 1720)[0]
            target_y_pixel = self.convert_to_pixels_y([current_target.position.z], 0.1, 0.5, 200, 1000)[0]
            return (int(target_x_pixel), int(target_y_pixel))
        return self.final_target_pixel

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
                    rospy.loginfo("Reached start position. Ready for multi-trajectory motion.")
            
            self.wait_thread = threading.Thread(target=wait_for_move_completion)
            self.wait_thread.start()

    def record_realtime_position(self):
        self.trail_points.append((int(self.x), int(self.y)))
        
        # Check distance to current target for success detection
        current_target_pixel = self.get_current_target_pixel()
        if current_target_pixel:
            distance = np.sqrt((self.x - current_target_pixel[0])**2 + 
                             (self.y - current_target_pixel[1])**2)
            self.min_distance_to_target = min(self.min_distance_to_target, distance)
            
            # Check if we've reached the current waypoint
            if distance <= self.success_radius and not self.trajectory_paused:
                rospy.loginfo(f"Reached waypoint {self.current_waypoint_index + 1}/{len(self.spline_waypoints)}!")
                self.completed_waypoints.append(current_target_pixel)
                
                # Check if this is the final waypoint
                if self.current_waypoint_index >= len(self.spline_waypoints) - 1:
                    self.success_achieved = True
                    self.trajectory_complete = True
                    rospy.loginfo("SUCCESS! Completed all waypoints!")
                else:
                    # Pause for next trajectory segment
                    self.trajectory_paused = True

    def start_robot_motion(self):
        if self.robot_ready_to_move and not self.robot_motion_active:
            self.robot_motion_active = True
            self.current_waypoint_index = 0
            # Start the timer
            self.timer_started = True
            self.start_time = time.time()
            rospy.loginfo("Starting multi-trajectory low impedance motion... Timer started!")
            
            def motion_wrapper():
                self.ik_motion.execute_multi_trajectory_motion(self.spline_waypoints, self)
                # Mark trajectory as complete
                self.trajectory_complete = True
                self.evaluate_final_result()
                rospy.loginfo("All trajectories complete. Stopping visualization...")
            
            self.motion_thread = threading.Thread(target=motion_wrapper)
            self.motion_thread.start()

    def continue_to_next_waypoint(self):
        """Continue to the next waypoint in the sequence"""
        if self.trajectory_paused and self.current_waypoint_index < len(self.spline_waypoints) - 1:
            self.current_waypoint_index += 1
            self.trajectory_paused = False
            self.min_distance_to_target = float('inf')  # Reset for next waypoint
            rospy.loginfo(f"Continuing to waypoint {self.current_waypoint_index + 1}/{len(self.spline_waypoints)}")
            return True
        return False

    def evaluate_final_result(self):
        """Evaluate whether the robot successfully reached all targets"""
        rospy.loginfo(f"=== FINAL RESULT EVALUATION ===")
        rospy.loginfo(f"Completed waypoints: {len(self.completed_waypoints)}/{len(self.spline_waypoints)}")
        rospy.loginfo(f"Final time: {self.elapsed_time:.2f} seconds")
        
        if self.success_achieved:
            rospy.loginfo("✅ SUCCESS: Robot completed all waypoints!")
        else:
            rospy.logwarn("❌ PARTIAL SUCCESS: Robot did not complete all waypoints")
        
        return self.success_achieved

    def draw_target_indicator(self):
        """Draw the current target position and success radius"""
        current_target_pixel = self.get_current_target_pixel()
        if current_target_pixel:
            # Draw success radius circle
            pygame.draw.circle(self.window, (0, 255, 0), current_target_pixel, self.success_radius, 3)
            
            # Draw target center
            pygame.draw.circle(self.window, (255, 255, 0), current_target_pixel, 10)
            
            # Draw crosshair
            cross_size = 15
            pygame.draw.line(self.window, (255, 255, 0), 
                           (current_target_pixel[0] - cross_size, current_target_pixel[1]),
                           (current_target_pixel[0] + cross_size, current_target_pixel[1]), 3)
            pygame.draw.line(self.window, (255, 255, 0),
                           (current_target_pixel[0], current_target_pixel[1] - cross_size),
                           (current_target_pixel[0], current_target_pixel[1] + cross_size), 3)
        
        # Draw completed waypoints in green
        for completed_waypoint in self.completed_waypoints:
            pygame.draw.circle(self.window, (0, 255, 0), completed_waypoint, 15)

    def update_timer(self):
        """Update the elapsed time if timer is running"""
        if self.timer_started and self.start_time is not None and not self.trajectory_complete:
            self.elapsed_time = time.time() - self.start_time

    def draw_timer(self):
        """Draw the timer on the screen"""
        if self.timer_started:
            # Format time as MM:SS.SS
            minutes = int(self.elapsed_time // 60)
            seconds = self.elapsed_time % 60
            time_text = f"{minutes:02d}:{seconds:05.2f}"
            
            # Create timer surface
            timer_surface = self.font.render(time_text, True, (255, 255, 255))  # White text
            timer_rect = timer_surface.get_rect()
            timer_rect.topright = (1900, 20)  # Top right corner
            
            # Draw semi-transparent background for timer
            background_rect = pygame.Rect(timer_rect.left - 10, timer_rect.top - 5, 
                                        timer_rect.width + 20, timer_rect.height + 10)
            overlay = pygame.Surface((background_rect.width, background_rect.height))
            overlay.set_alpha(128)  # Semi-transparent
            overlay.fill((0, 0, 0))  # Black background
            self.window.blit(overlay, (background_rect.left, background_rect.top))
            
            # Draw timer text
            self.window.blit(timer_surface, timer_rect)

    def draw_status_info(self):
        """Draw current distance and waypoint progress"""
        if self.robot_motion_active:
            current_target_pixel = self.get_current_target_pixel()
            if current_target_pixel:
                # Calculate current distance
                current_distance = np.sqrt((self.x - current_target_pixel[0])**2 + 
                                         (self.y - current_target_pixel[1])**2)
                
                # Status text
                waypoint_text = f"Waypoint: {self.current_waypoint_index + 1}/{len(self.spline_waypoints)}"
                distance_text = f"Distance: {current_distance:.1f}px"
                
                if self.trajectory_paused:
                    status_text = "PAUSED - Press SPACEBAR to continue"
                    text_color = (255, 255, 0)  # Yellow
                elif current_distance <= self.success_radius:
                    status_text = "IN TARGET ZONE!"
                    text_color = (0, 255, 0)  # Green
                else:
                    status_text = "Moving to target..."
                    text_color = (255, 255, 255)  # White
                
                # Draw status lines
                y_offset = 20
                for text, color in [(waypoint_text, (255, 255, 255)), 
                                   (distance_text, (255, 255, 255)), 
                                   (status_text, text_color)]:
                    status_surface = self.small_font.render(text, True, color)
                    status_rect = status_surface.get_rect()
                    status_rect.topleft = (20, y_offset)
                    
                    # Background
                    background_rect = pygame.Rect(status_rect.left - 10, status_rect.top - 5,
                                                status_rect.width + 20, status_rect.height + 10)
                    overlay = pygame.Surface((background_rect.width, background_rect.height))
                    overlay.set_alpha(128)
                    overlay.fill((0, 0, 0))
                    self.window.blit(overlay, (background_rect.left, background_rect.top))
                    
                    self.window.blit(status_surface, status_rect)
                    y_offset += 40

    def draw_instructions(self):
        """Draw instructions on the screen"""
        if not self.robot_ready_to_move and not self.spacebar_disabled:
            instruction_text = "Press SPACEBAR to move to start position"
        elif self.robot_ready_to_move and not self.robot_motion_active:
            instruction_text = "Press SPACEBAR to start multi-trajectory motion"
        elif self.trajectory_paused:
            instruction_text = "Press SPACEBAR to continue to next waypoint"
        elif self.robot_motion_active:
            instruction_text = "Robot in motion - You can manually guide the arm!"
        else:
            instruction_text = ""
        
        if instruction_text:
            text_surface = self.small_font.render(instruction_text, True, (255, 255, 255))
            text_rect = text_surface.get_rect()
            text_rect.centerx = self.window.get_width() // 2
            text_rect.bottom = self.window.get_height() - 20
            
            # Draw semi-transparent background
            background_rect = pygame.Rect(text_rect.left - 10, text_rect.top - 5,
                                        text_rect.width + 20, text_rect.height + 10)
            overlay = pygame.Surface((background_rect.width, background_rect.height))
            overlay.set_alpha(128)
            overlay.fill((0, 0, 0))
            self.window.blit(overlay, (background_rect.left, background_rect.top))
            
            # Draw instruction text
            self.window.blit(text_surface, text_rect)

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
                        elif not self.robot_motion_active:  # Second press - start motion
                            self.start_robot_motion()
                        elif self.trajectory_paused:  # Continue to next waypoint
                            self.continue_to_next_waypoint()

            # Update timer
            self.update_timer()

            self.window.fill(self.background_color)
            
            # Draw spline path as guide
            for i, point in enumerate(self.smooth_points):
                color = (100, 100, 100) if i < len(self.completed_waypoints) * (len(self.smooth_points) // len(self.spline_waypoints)) else (255, 0, 0)
                pygame.draw.circle(self.window, color, (int(point[0]), int(point[1])), 3)
            
            # Draw target indicators
            self.draw_target_indicator()
            
            # Draw real-time trail when following trajectory
            if self.robot_motion_active and self.robot_ready_to_move:
                self.record_realtime_position()
                for point in self.trail_points:
                    pygame.draw.circle(self.window, (0, 0, 0), (point[0], point[1]), 5)

            with self.lock:
                pygame.draw.circle(self.window, self.icon_color, (int(self.x), int(self.y)), self.icon_radius)
            
            # Draw timer, status, and instructions
            self.draw_timer()
            self.draw_status_info()
            self.draw_instructions()
            
            pygame.display.flip()
            clock.tick(60)

        # Show final time if trajectory completed
        if self.trajectory_complete and self.timer_started:
            final_result = "SUCCESS" if self.success_achieved else "PARTIAL SUCCESS"
            rospy.loginfo(f"Final completion time: {self.elapsed_time:.2f} seconds - {final_result}")

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
        
        # Very low stiffness for manual guidance
        self.low_stiffness = [50.0, 2.0, 2.0, 5.0, 5.0, 5.0]  # Increased X stiffness to prevent drift
        
        rospy.loginfo("IK and Multi-Trajectory Low Impedance Motion setup complete.")

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

    def generate_spline_waypoints(self, start_pose, end_pose, num_points=10):  # Reduced points for clearer waypoints
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

    def enable_low_impedance(self):
        """Enable low impedance mode for manual guidance"""
        rospy.loginfo("Enabling low impedance mode...")
        
        # Suppress collision avoidance
        self.collision_suppress_pub.publish(Empty())
        
        # Set very low stiffness
        low_impedance_msg = create_interaction_msg(self.low_stiffness)
        self.impedance_pub.publish(low_impedance_msg)
        
        rospy.loginfo("Low impedance mode enabled. Robot can be manually guided.")

    def disable_impedance_control(self):
        """Disable impedance control and return to position mode"""
        rospy.loginfo("Disabling impedance control...")
        position_control_msg = create_position_control_msg()
        for _ in range(5):  # Publish multiple times to ensure mode switch
            self.impedance_pub.publish(position_control_msg)
            rospy.sleep(0.2)
        rospy.loginfo("Returned to position control mode.")

    def execute_single_trajectory_segment(self, start_pose, target_pose):
        """
        Execute a single trajectory segment between two poses with low impedance
        """
        try:
            # Enable low impedance mode
            self.enable_low_impedance()
            rospy.sleep(1.0)  # Allow impedance mode to activate
            
            # Set slow motion parameters
            wpt_opts = MotionWaypointOptions(
                max_linear_speed=0.005,      # 5mm/s
                max_linear_accel=0.005,      # Gentle acceleration
                max_rotational_speed=0.005,
                max_rotational_accel=0.005,
                max_joint_speed_ratio=0.1    # Slow joint motion
            )
            
            # Calculate IK for target position
            joint_angles = self.calculate_ik(target_pose)
            if joint_angles is None:
                rospy.logerr("Failed to calculate IK for target position")
                return False
            
            # Create waypoint to target position
            waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
            waypoint.set_joint_angles(joint_angles, "right_hand")
            
            # Clear any existing trajectory and add the waypoint
            self.traj.clear_waypoints()
            self.traj.append_waypoint(waypoint.to_msg())
            
            rospy.loginfo(f"Moving to waypoint with low impedance...")
            
            # Keep publishing low impedance commands during motion
            def maintain_low_impedance():
                rate = rospy.Rate(10)  # 10 Hz
                while not rospy.is_shutdown():
                    self.collision_suppress_pub.publish(Empty())
                    low_impedance_msg = create_interaction_msg(self.low_stiffness)
                    self.impedance_pub.publish(low_impedance_msg)
                    rate.sleep()
            
            # Start impedance maintenance thread
            impedance_thread = threading.Thread(target=maintain_low_impedance)
            impedance_thread.daemon = True
            impedance_thread.start()
            
            # Execute the trajectory
            result = self.traj.send_trajectory()
            
            if result is None:
                rospy.logerr("Trajectory FAILED to send")
                return False
            elif result.result:
                rospy.loginfo("Successfully completed trajectory segment!")
                return True
            else:
                rospy.logerr(f"Trajectory execution failed with error {result.errorId}")
                return False
            
        except Exception as e:
            rospy.logerr(f"Error during trajectory segment: {e}")
            return False

    def execute_multi_trajectory_motion(self, spline_waypoints, visualizer):
        """
        Execute multiple trajectory segments, moving from waypoint to waypoint
        """
        try:
            rospy.loginfo(f"Starting multi-trajectory motion with {len(spline_waypoints)} waypoints")
            
            # Get current position as starting point
            current_pose = start_pose  # You might want to get actual current pose here
            
            for i, target_waypoint in enumerate(spline_waypoints):
                rospy.loginfo(f"=== Executing trajectory segment {i+1}/{len(spline_waypoints)} ===")
                
                # Execute trajectory to this waypoint
                success = self.execute_single_trajectory_segment(current_pose, target_waypoint)
                
                if not success:
                    rospy.logerr(f"Failed to reach waypoint {i+1}")
                    break
                
                # Wait for visualizer to detect we've reached the waypoint
                rospy.loginfo("Waiting for waypoint confirmation...")
                timeout_counter = 0
                max_timeout = 300  # 30 seconds at 10Hz
                
                while not visualizer.trajectory_paused and not visualizer.trajectory_complete and timeout_counter < max_timeout:
                    rospy.sleep(0.1)
                    timeout_counter += 1
                
                if timeout_counter >= max_timeout:
                    rospy.logwarn(f"Timeout waiting for waypoint {i+1} confirmation")
                
                # If this isn't the last waypoint, wait for user to continue
                if i < len(spline_waypoints) - 1:
                    rospy.loginfo("Waiting for user to continue to next waypoint...")
                    while visualizer.trajectory_paused and not rospy.is_shutdown():
                        rospy.sleep(0.1)
                
                # Update current pose for next segment
                current_pose = target_waypoint
                
                if visualizer.trajectory_complete:
                    break
            
            rospy.loginfo("Multi-trajectory motion completed!")
            
        except Exception as e:
            rospy.logerr(f"Error during multi-trajectory motion: {e}")
        
        finally:
            # Always disable impedance control when done
            rospy.sleep(2.0)  # Allow motion to complete
            self.disable_impedance_control()


if __name__ == '__main__':
    try:
        sawyerVisualizer = SawyerVisualizer()
        sawyerVisualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt detected. Exiting...")