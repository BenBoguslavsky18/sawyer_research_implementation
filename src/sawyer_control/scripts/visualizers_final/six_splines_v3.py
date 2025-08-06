#!/usr/bin/env python

#  Sawyer path visualizer with 30-second time limit and pygame restart between levels
#  Modified to run each spline twice: first normal, then with trajectory tracking

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
import sys
import subprocess
import os


'''IMPORTANT VARIABLES'''
x_screen_limit_left: int = 200
x_screen_limit_right: int = 1720
y_screen_limit_top: int = 200
y_screen_limit_bottom: int = 1000

# Game timing constants
GAME_DURATION = 30.0  # 30 seconds
NUM_LEVELS = 6  # Number of spline challenges

# Define 6 different spline challenges with varying difficulty
SPLINE_CHALLENGES = [
    # Level 1: Simple curved spline (from first document)
    {
        'start': Pose(position=Point(x=0.75, y=-0.3, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'end': Pose(position=Point(x=0.75, y=0.5, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'control_points': [[0.75, -0.3, 0.3], [0.75, -0.1, 0.4], [0.75, 0.1, 0.2], [0.75, 0.3, 0.4], [0.75, 0.5, 0.3]],
        'name': "Level 1: Curved Spline",
        'type': 'spline'
    },
    # Level 2: Different curved spline (from second document) 
    {
        'start': Pose(position=Point(x=0.75, y=-0.3, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'end': Pose(position=Point(x=0.75, y=0.5, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'control_points': [[0.75, -0.3, 0.3], [0.75, -0.1, 0.2], [0.75, 0.1, 0.4], [0.75, 0.3, 0.2], [0.75, 0.5, 0.3]],
        'name': "Level 2: Alternate Spline",
        'type': 'spline'
    },
    # Level 3: Straight line segments (from third document)
    {
        'start': Pose(position=Point(x=0.75, y=-0.3, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'end': Pose(position=Point(x=0.75, y=0.5, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'control_points': [[0.75, -0.3, 0.3], [0.75, -0.1, 0.5], [0.75, 0.1, 0.15], [0.75, 0.3, 0.35], [0.75, 0.5, 0.3]],
        'name': "Level 3: Linear Segments",
        'type': 'linear'
    },
    # Level 4: Complex maze (from fourth document)
    {
        'start': Pose(position=Point(x=0.75, y=-0.3, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'end': Pose(position=Point(x=0.75, y=0.5, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'control_points': [
            [0.75, -0.3, 0.3], [0.75, -0.25, 0.4], [0.8, -0.2, 0.35], [0.85, -0.15, 0.3], [0.9, -0.1, 0.25], 
            [0.85, -0.05, 0.2], [0.8, 0.0, 0.15], [0.75, 0.05, 0.18], [0.7, 0.1, 0.22], [0.65, 0.15, 0.28],
            [0.6, 0.2, 0.32], [0.55, 0.25, 0.35], [0.6, 0.3, 0.4], [0.65, 0.35, 0.38], [0.7, 0.4, 0.35],
            [0.72, 0.42, 0.32], [0.74, 0.46, 0.31], [0.75, 0.48, 0.3], [0.75, 0.5, 0.3]
        ],
        'name': "Level 4: Complex Maze",
        'type': 'linear'
    },
    # Level 5: Simple maze (from fifth document)
    {
        'start': Pose(position=Point(x=0.75, y=-0.3, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'end': Pose(position=Point(x=0.75, y=0.5, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'control_points': [
            [0.75, -0.3, 0.3], [0.75, -0.2, 0.4], [0.75, 0.0, 0.4], [0.75, 0.1, 0.2], 
            [0.75, 0.2, 0.35], [0.75, 0.3, 0.2], [0.75, 0.4, 0.35], [0.75, 0.5, 0.3]
        ],
        'name': "Level 5: Simple Maze",
        'type': 'linear'
    },
    # Level 6: Zigzag with coverage tracking (from sixth document)
    {
        'start': Pose(position=Point(x=0.75, y=-0.3, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'end': Pose(position=Point(x=0.75, y=0.5, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'control_points': [
            [0.75, -0.3, 0.3], [0.75, -0.2, 0.4], [0.75, 0.0, 0.4], [0.75, 0.1, 0.2], 
            [0.75, 0.2, 0.35], [0.75, 0.3, 0.2], [0.75, 0.4, 0.35], [0.75, 0.5, 0.3]
        ],
        'name': "Level 6: Zigzag Expert",
        'type': 'linear'
    }
]


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


def get_current_level():
    """Get the current level from command line arguments or environment variable"""
    # Check command line argument first
    if len(sys.argv) > 1:
        try:
            return int(sys.argv[1])
        except ValueError:
            pass
    
    # Check environment variable
    level_env = os.environ.get('SAWYER_LEVEL', '0')
    try:
        return int(level_env)
    except ValueError:
        return 0


def get_current_run():
    """Get the current run (1 or 2) from environment variable"""
    run_env = os.environ.get('SAWYER_RUN', '1')
    try:
        return int(run_env)
    except ValueError:
        return 1


def restart_for_next_run_or_level(current_level, current_run):
    """Restart the program for the next run or level"""
    if current_run == 1:
        # Start run 2 (tracking mode) of the same level
        next_level = current_level
        next_run = 2
        rospy.loginfo(f"Restarting for tracking mode of level {current_level + 1}...")
    else:
        # Move to next level, run 1
        next_level = current_level + 1
        next_run = 1
        
        if next_level >= NUM_LEVELS:
            rospy.loginfo("🎉 ALL LEVELS AND RUNS COMPLETED! 🎉")
            return False
        
        rospy.loginfo(f"Restarting for level {next_level + 1}, run 1...")
    
    # Set environment variables for next run/level
    env = os.environ.copy()
    env['SAWYER_LEVEL'] = str(next_level)
    env['SAWYER_RUN'] = str(next_run)
    
    # Restart the script
    script_path = os.path.abspath(__file__)
    subprocess.Popen([sys.executable, script_path, str(next_level)], env=env)
    
    return True


'''Visualizer Display'''
class SawyerVisualizer:
    def __init__(self, level_index=0, run_number=1):
        rospy.init_node('impedance_spline_tracer', anonymous=True)

        pygame.init()
        self.window = pygame.display.set_mode((1920, 1420))
        self.background_color = (2, 114, 212)
        
        # Initialize font for timer display
        pygame.font.init()
        self.font = pygame.font.Font(None, 72)  # Large font for timer
        self.small_font = pygame.font.Font(None, 36)  # Smaller font for instructions
        self.big_font = pygame.font.Font(None, 120)  # Extra large font for WIN/LOSE messages

        # Level and run tracking
        self.current_level = level_index
        self.current_run = run_number  # 1 = normal, 2 = tracking
        self.total_levels = NUM_LEVELS
        self.current_challenge = SPLINE_CHALLENGES[level_index]
        
        # Update window caption with current level and run
        run_type = "Normal Mode" if run_number == 1 else "Tracking Mode"
        pygame.display.set_caption(f"Sawyer Challenge - {self.current_challenge['name']} - {run_type}")
        
        rospy.loginfo(f"Starting {self.current_challenge['name']} - {run_type} (Level {level_index + 1}/{NUM_LEVELS}, Run {run_number}/2)")
        
        # Create fresh motion interface for this level
        rospy.loginfo("Creating fresh robot motion interface...")
        self.ik_motion = IKMotionWaypoint()
        
        # Generate spline for current challenge
        start_pose = self.current_challenge['start']
        end_pose = self.current_challenge['end']
        control_points = self.current_challenge['control_points']
        spline_type = self.current_challenge.get('type', 'spline')
        
        self.spline_controlpoints, self.spline_waypoints, self.display_waypoints = self.ik_motion.generate_spline_waypoints_custom(
            start_pose, end_pose, control_points, spline_type)

        x_coords = [p[0] for p in self.display_waypoints]
        y_coords = [p[1] for p in self.display_waypoints]
        # CRITICAL FIX: Use the same coordinate ranges as reference code
        x_coords = self.convert_to_pixels_x(x_coords, -0.3, 0.5, 200, 1720)  # Changed from -0.4, 0.6
        y_coords = self.convert_to_pixels_y(y_coords, 0.1, 0.5, 200, 1000)
        self.smooth_points = list(zip(x_coords, y_coords))
        self.trail_points = []
        
        # Calculate final target position in pixels using same coordinate ranges
        final_y = end_pose.position.y
        final_z = end_pose.position.z
        target_x_pixel = self.convert_to_pixels_x([final_y], -0.3, 0.5, 200, 1720)[0]  # Changed from -0.4, 0.6
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

        # Game state variables
        self.game_state = "waiting_to_start"  # "waiting_to_start", "moving_to_start", "ready_to_play", "playing", "game_over"
        self.robot_ready_to_move = False
        self.spacebar_disabled = False
        
        # Timer variables
        self.game_timer_active = False
        self.game_start_time = None
        self.remaining_time = GAME_DURATION
        self.game_timer_thread = None
        
        # Success detection variables
        self.success_radius = 50  # pixels - radius for success detection
        self.min_distance_to_target = float('inf')  # Track closest approach
        self.success_achieved = False
        self.game_won = False
        self.game_lost = False
        
        # Game result tracking
        self.final_distance = None
        self.game_completion_time = None
        self.show_results_timer = None
        self.auto_close_delay = 5.0  # Show results for 5 seconds before closing
        self.restart_next_run = False  # Flag to trigger restart

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
            # Use the same scaling as reference code
            screen_x = (msg.pose.position.y * self.scale_x) + 770  # Reference uses 770 offset
            screen_y = (-msg.pose.position.z * self.scale_y) + 1180
            self.x = max(0, min(1920 - self.icon_radius, screen_x))
            self.y = max(0, min(1080 - self.icon_radius, screen_y))  # Note: 1080 not 1420

    def move_to_start_position(self):
        """Move robot to start position and prepare for game"""
        run_type = "Normal Mode" if self.current_run == 1 else "Tracking Mode"
        rospy.loginfo(f"Moving to start position for {self.current_challenge['name']} - {run_type}...")
        self.game_state = "moving_to_start"
        
        def move_and_signal():
            # Get start pose for current challenge
            start_pose = self.current_challenge['start']
            rospy.loginfo(f"Target start position: x={start_pose.position.x}, y={start_pose.position.y}, z={start_pose.position.z}")
            
            # DEBUG: Show where this should appear on screen
            start_x_pixel = self.convert_to_pixels_x([start_pose.position.y], -0.3, 0.5, 200, 1720)[0]
            start_y_pixel = self.convert_to_pixels_y([start_pose.position.z], 0.1, 0.5, 200, 1000)[0]
            rospy.loginfo(f"Expected start position on screen: x={start_x_pixel:.1f}, y={start_y_pixel:.1f}")
            
            # Move to start position
            success = self.ik_motion.move_to_pose(start_pose)
            
            if not rospy.is_shutdown():
                if success is not False:
                    self.robot_ready_to_move = True
                    self.game_state = "ready_to_play"
                    self.spacebar_disabled = False
                    rospy.loginfo(f"Successfully reached start position. Ready for {run_type}!")
                    # DEBUG: Show where robot actually is on screen
                    rospy.loginfo(f"Actual robot position on screen: x={self.x:.1f}, y={self.y:.1f}")
                else:
                    rospy.logerr("Failed to reach start position. Please try again.")
                    self.game_state = "waiting_to_start"
                    self.spacebar_disabled = False
        
        self.start_position_thread = threading.Thread(target=move_and_signal)
        self.start_position_thread.daemon = True
        self.start_position_thread.start()

    def start_game_timer(self):
        """Start the 30-second game timer"""
        def timer_countdown():
            run_type = "Normal Mode" if self.current_run == 1 else "Tracking Mode"
            rospy.loginfo(f"=== 30-SECOND CHALLENGE STARTED - {run_type} ===")
            self.game_start_time = time.time()
            self.remaining_time = GAME_DURATION
            
            while self.remaining_time > 0 and not self.game_won and not rospy.is_shutdown():
                time.sleep(0.1)
                elapsed = time.time() - self.game_start_time
                self.remaining_time = max(0, GAME_DURATION - elapsed)
            
            if not self.game_won and self.remaining_time <= 0:
                self.game_lost = True
                rospy.loginfo("=== TIME'S UP! GAME OVER ===")
            
            self.end_game()
        
        self.game_timer_thread = threading.Thread(target=timer_countdown)
        self.game_timer_thread.daemon = True
        self.game_timer_thread.start()

    def start_game(self):
        """Initialize the game - start trajectory and timer simultaneously"""
        if self.game_state == "ready_to_play":
            self.game_state = "playing"
            self.trail_points = []
            self.success_achieved = False
            self.game_won = False
            self.game_lost = False
            self.min_distance_to_target = float('inf')
            
            self.game_timer_active = True
            self.start_game_timer()
            
            # Choose trajectory type based on current run
            if self.current_run == 1:
                # Run 1: Normal low impedance trajectory (just to end point)
                rospy.loginfo("Starting low impedance trajectory to end point (Normal Mode)...")
                def start_trajectory():
                    self.ik_motion.execute_low_impedance_trajectory(self.spline_waypoints)
            else:
                # Run 2: Tracking trajectory (through all spline points)
                rospy.loginfo("Starting tracking trajectory through all spline points (Tracking Mode)...")
                def start_trajectory():
                    self.ik_motion.execute_tracking_trajectory(self.spline_waypoints)
            
            self.trajectory_thread = threading.Thread(target=start_trajectory)
            self.trajectory_thread.daemon = True
            self.trajectory_thread.start()

    def check_success(self):
        """Check if player has reached the target"""
        if self.game_state == "playing" and not self.game_won:
            distance = np.sqrt((self.x - self.final_target_pixel[0])**2 + 
                             (self.y - self.final_target_pixel[1])**2)
            self.min_distance_to_target = min(self.min_distance_to_target, distance)
            
            if distance <= self.success_radius:
                self.game_won = True
                self.success_achieved = True
                self.game_completion_time = time.time() - self.game_start_time
                run_type = "Normal Mode" if self.current_run == 1 else "Tracking Mode"
                rospy.loginfo(f"=== {self.current_challenge['name']} - {run_type} COMPLETED in {self.game_completion_time:.2f} seconds ===")
                self.end_game()

    def end_game(self):
        """End the game and provide final results"""
        if self.game_state == "playing":
            self.game_state = "game_over"
            self.game_timer_active = False
            
            # Calculate final distance
            self.final_distance = np.sqrt((self.x - self.final_target_pixel[0])**2 + 
                                        (self.y - self.final_target_pixel[1])**2)
            
            # ENHANCED SAFETY: More aggressive trajectory stopping
            rospy.loginfo("EMERGENCY STOP: Halting all robot motion immediately...")
            
            # Clear waypoints multiple times to ensure trajectory stops
            for i in range(3):
                self.ik_motion.traj.clear_waypoints()
                rospy.sleep(0.1)
            
            # Try to lock robot at current position to prevent movement
            try:
                current_joint_angles = self.ik_motion._limb.joint_angles()
                hold_wpt_opts = MotionWaypointOptions(
                    max_linear_speed=0.001,
                    max_linear_accel=0.001,
                    max_rotational_speed=0.001,
                    max_rotational_accel=0.001,
                    max_joint_speed_ratio=0.01
                )
                hold_waypoint = MotionWaypoint(options=hold_wpt_opts.to_msg(), limb=self.ik_motion._limb)
                hold_waypoint.set_joint_angles(current_joint_angles, "right_hand")
                
                self.ik_motion.traj.clear_waypoints()
                self.ik_motion.traj.append_waypoint(hold_waypoint.to_msg())
                rospy.loginfo("Robot locked at current position")
            except Exception as e:
                rospy.logwarn(f"Could not lock robot position: {e}")
                
            rospy.loginfo("Robot motion stopped. Now disabling impedance control...")
            
            # Small delay to ensure trajectory is completely cleared
            rospy.sleep(0.5)
            
            # Now disable impedance control to return to normal position control
            self.ik_motion.disable_impedance_control()
            
            # Print results
            run_type = "Normal Mode" if self.current_run == 1 else "Tracking Mode"
            rospy.loginfo("=== RUN RESULTS ===")
            if self.game_won:
                rospy.loginfo(f"✅ {self.current_challenge['name']} - {run_type} COMPLETED in {self.game_completion_time:.2f} seconds")
                
                # Check if this is the final run
                if self.current_level >= self.total_levels - 1 and self.current_run == 2:
                    rospy.loginfo("🎉 FINAL LEVEL AND RUN COMPLETED! 🎉")
                elif self.current_run == 1:
                    rospy.loginfo(f"Preparing to start tracking mode for level {self.current_level + 1}...")
                    self.restart_next_run = True
                else:
                    rospy.loginfo(f"Preparing to advance to level {self.current_level + 2}...")
                    self.restart_next_run = True
            else:
                rospy.loginfo(f"❌ {self.current_challenge['name']} - {run_type} FAILED! Time ran out.")
            
            rospy.loginfo(f"Final distance to target: {self.final_distance:.1f} pixels")
            rospy.loginfo(f"Closest approach: {self.min_distance_to_target:.1f} pixels")
            
            # Start auto-close timer
            self.show_results_timer = time.time()

    def record_realtime_position(self):
        """Record the robot's position and check for success"""
        if self.game_state == "playing":
            self.trail_points.append((int(self.x), int(self.y)))
            self.check_success()

    def draw_target_indicator(self):
        """Draw the target position and success radius"""
        if self.final_target_pixel:
            # Draw success radius circle
            pygame.draw.circle(self.window, (0, 255, 0), self.final_target_pixel, self.success_radius, 3)
            
            # Draw target center
            pygame.draw.circle(self.window, (255, 255, 0), self.final_target_pixel, 10)
            
            # Draw crosshair
            cross_size = 15
            pygame.draw.line(self.window, (255, 255, 0), 
                           (self.final_target_pixel[0] - cross_size, self.final_target_pixel[1]),
                           (self.final_target_pixel[0] + cross_size, self.final_target_pixel[1]), 3)
            pygame.draw.line(self.window, (255, 255, 0),
                           (self.final_target_pixel[0], self.final_target_pixel[1] - cross_size),
                           (self.final_target_pixel[0], self.final_target_pixel[1] + cross_size), 3)

    def draw_timer(self):
        """Draw the game timer"""
        if self.game_timer_active and self.game_state == "playing":
            minutes = int(self.remaining_time // 60)
            seconds = int(self.remaining_time % 60)
            time_text = f"{minutes:02d}:{seconds:02d}"
            
            if self.remaining_time <= 5:
                color = (255, 0, 0)
            elif self.remaining_time <= 10:
                color = (255, 255, 0)
            else:
                color = (255, 255, 255)
            
            timer_surface = self.font.render(time_text, True, color)
            timer_rect = timer_surface.get_rect()
            timer_rect.topright = (1900, 20)
            
            background_rect = pygame.Rect(timer_rect.left - 10, timer_rect.top - 5, 
                                        timer_rect.width + 20, timer_rect.height + 10)
            overlay = pygame.Surface((background_rect.width, background_rect.height))
            overlay.set_alpha(128)
            overlay.fill((0, 0, 0))
            self.window.blit(overlay, (background_rect.left, background_rect.top))
            
            self.window.blit(timer_surface, timer_rect)

    def draw_status_info(self):
        """Draw current distance and game status"""
        if self.game_state == "playing" and self.final_target_pixel:
            current_distance = np.sqrt((self.x - self.final_target_pixel[0])**2 + 
                                     (self.y - self.final_target_pixel[1])**2)
            
            status_text = f"Distance: {current_distance:.1f}px"
            if self.success_achieved:
                status_text += " - YOU WIN!"
                text_color = (0, 255, 0)
            elif current_distance <= self.success_radius:
                status_text += " - IN TARGET!"
                text_color = (255, 255, 0)
            else:
                text_color = (255, 255, 255)
            
            status_surface = self.small_font.render(status_text, True, text_color)
            status_rect = status_surface.get_rect()
            status_rect.topleft = (20, 20)
            
            background_rect = pygame.Rect(status_rect.left - 10, status_rect.top - 5,
                                        status_rect.width + 20, status_rect.height + 10)
            overlay = pygame.Surface((background_rect.width, background_rect.height))
            overlay.set_alpha(128)
            overlay.fill((0, 0, 0))
            self.window.blit(overlay, (background_rect.left, background_rect.top))
            
            self.window.blit(status_surface, status_rect)

    def draw_level_info(self):
        """Draw current level and run information"""
        run_type = "Normal" if self.current_run == 1 else "Tracking"
        level_text = f"Level {self.current_level + 1}/{self.total_levels}: {self.current_challenge['name']} - {run_type} Mode"
        level_surface = self.small_font.render(level_text, True, (255, 255, 255))
        level_rect = level_surface.get_rect()
        level_rect.topleft = (20, 60)
        
        background_rect = pygame.Rect(level_rect.left - 10, level_rect.top - 5,
                                    level_rect.width + 20, level_rect.height + 10)
        overlay = pygame.Surface((background_rect.width, background_rect.height))
        overlay.set_alpha(128)
        overlay.fill((0, 0, 0))
        self.window.blit(overlay, (background_rect.left, background_rect.top))
        
        self.window.blit(level_surface, level_rect)

    def draw_game_result(self):
        """Draw large WIN/LOSE message in center of screen"""
        if self.game_state == "game_over" or (self.game_state == "playing" and (self.game_won or self.game_lost)):
            if self.game_won:
                if self.current_level >= self.total_levels - 1 and self.current_run == 2:
                    result_text = "ALL COMPLETE!"
                    text_color = (255, 215, 0)  # Gold for final completion
                elif self.current_run == 1:
                    result_text = "RUN 1 COMPLETE!"
                    text_color = (0, 255, 0)  # Green for run 1
                else:
                    result_text = "RUN 2 COMPLETE!"
                    text_color = (0, 255, 0)  # Green for run 2
                completion_text = f"Time: {self.game_completion_time:.1f}s"
            elif self.game_lost:
                result_text = "TIME'S UP!"
                text_color = (255, 0, 0)
                completion_text = f"Distance: {self.final_distance:.1f}px"
            else:
                return
            
            # Draw main result text
            result_surface = self.big_font.render(result_text, True, text_color)
            result_rect = result_surface.get_rect()
            result_rect.center = (self.window.get_width() // 2, self.window.get_height() // 2 - 50)
            
            # Background
            background_width = result_rect.width + 40
            background_height = 120
            background_rect = pygame.Rect(
                result_rect.centerx - background_width // 2,
                result_rect.centery - background_height // 2,
                background_width, background_height
            )
            overlay = pygame.Surface((background_width, background_height))
            overlay.set_alpha(180)
            overlay.fill((0, 0, 0))
            self.window.blit(overlay, (background_rect.left, background_rect.top))
            
            self.window.blit(result_surface, result_rect)
            
            # Draw completion details
            if hasattr(self, 'game_completion_time') and self.game_completion_time:
                detail_surface = self.font.render(completion_text, True, (255, 255, 255))
            elif hasattr(self, 'final_distance') and self.final_distance:
                detail_surface = self.font.render(completion_text, True, (255, 255, 255))
            else:
                detail_surface = None
                
            if detail_surface:
                detail_rect = detail_surface.get_rect()
                detail_rect.center = (self.window.get_width() // 2, result_rect.bottom + 30)
                
                # Background for detail text
                detail_bg_rect = pygame.Rect(detail_rect.left - 10, detail_rect.top - 5,
                                           detail_rect.width + 20, detail_rect.height + 10)
                detail_overlay = pygame.Surface((detail_bg_rect.width, detail_bg_rect.height))
                detail_overlay.set_alpha(150)
                detail_overlay.fill((0, 0, 0))
                self.window.blit(detail_overlay, (detail_bg_rect.left, detail_bg_rect.top))
                
                self.window.blit(detail_surface, detail_rect)

    def draw_instructions(self):
        """Draw instructions based on current game state"""
        instruction_text = ""
        run_type = "Normal Mode" if self.current_run == 1 else "Tracking Mode"
        
        if self.game_state == "waiting_to_start":
            instruction_text = f"Press SPACEBAR to move robot to start position ({run_type})"
        elif self.game_state == "moving_to_start":
            instruction_text = f"Moving to start position for {run_type}... Please wait"
        elif self.game_state == "ready_to_play":
            if self.current_run == 1:
                instruction_text = "Press SPACEBAR to start Normal Mode (low impedance to end point)"
            else:
                instruction_text = "Press SPACEBAR to start Tracking Mode (robot follows all spline points)"
        elif self.game_state == "playing":
            if self.game_won:
                instruction_text = "SUCCESS! You reached the target!"
            elif self.game_lost:
                instruction_text = "TIME'S UP! Game Over."
            else:
                if self.current_run == 1:
                    instruction_text = "NORMAL MODE: Guide robot to target! (Robot moves to end point)"
                else:
                    instruction_text = "TRACKING MODE: Guide robot to target! (Robot follows spline path)"
        elif self.game_state == "game_over":
            if self.game_won:
                if self.current_level >= self.total_levels - 1 and self.current_run == 2:
                    instruction_text = "🎉 ALL LEVELS AND RUNS COMPLETED! Window will close automatically..."
                else:
                    if self.show_results_timer:
                        remaining_time = self.auto_close_delay - (time.time() - self.show_results_timer)
                        if self.current_run == 1:
                            instruction_text = f"Starting Tracking Mode soon... ({remaining_time:.1f}s)"
                        else:
                            instruction_text = f"Next level starting soon... ({remaining_time:.1f}s)"
                    else:
                        if self.current_run == 1:
                            instruction_text = "Starting Tracking Mode soon..."
                        else:
                            instruction_text = "Next level starting soon..."
            else:
                instruction_text = "Game Over - Press SPACEBAR to try again"
        
        if instruction_text:
            text_surface = self.small_font.render(instruction_text, True, (255, 255, 255))
            text_rect = text_surface.get_rect()
            text_rect.centerx = self.window.get_width() // 2
            text_rect.bottom = self.window.get_height() - 20
            
            background_rect = pygame.Rect(text_rect.left - 10, text_rect.top - 5,
                                        text_rect.width + 20, text_rect.height + 10)
            overlay = pygame.Surface((background_rect.width, background_rect.height))
            overlay.set_alpha(128)
            overlay.fill((0, 0, 0))
            self.window.blit(overlay, (background_rect.left, background_rect.top))
            
            self.window.blit(text_surface, text_rect)

    def run(self):
        clock = pygame.time.Clock()
        running = True

        while running and not rospy.is_shutdown():
            # Check if we should auto-close and restart for next run/level
            if (self.game_state == "game_over" and 
                self.show_results_timer and 
                time.time() - self.show_results_timer >= self.auto_close_delay):
                
                if self.restart_next_run:
                    if self.current_level >= self.total_levels - 1 and self.current_run == 2:
                        # All levels and runs completed
                        rospy.loginfo("All levels and runs completed! Closing game...")
                        running = False
                    else:
                        # Restart for next run or level
                        rospy.loginfo("Closing pygame and restarting for next run/level...")
                        restart_for_next_run_or_level(self.current_level, self.current_run)
                        running = False
                else:
                    # Failed run, just close
                    running = False
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        if self.game_state == "waiting_to_start":
                            self.spacebar_disabled = True
                            self.move_to_start_position()
                        elif self.game_state == "ready_to_play":
                            self.start_game()
                        elif self.game_state == "game_over" and not self.game_won:
                            # Restart the current run by restarting the program
                            rospy.loginfo("Restarting current run...")
                            if self.current_run == 1:
                                restart_level = self.current_level
                                restart_run = 1
                            else:
                                restart_level = self.current_level  
                                restart_run = 2
                            
                            env = os.environ.copy()
                            env['SAWYER_LEVEL'] = str(restart_level)
                            env['SAWYER_RUN'] = str(restart_run)
                            script_path = os.path.abspath(__file__)
                            subprocess.Popen([sys.executable, script_path, str(restart_level)], env=env)
                            running = False

            self.window.fill(self.background_color)
            
            # Draw spline path as guide
            for point in self.smooth_points:
                pygame.draw.circle(self.window, (255, 0, 0), (int(point[0]), int(point[1])), 5)
            
            # Draw target indicator
            self.draw_target_indicator()
            
            # Draw real-time trail during gameplay
            if self.game_state == "playing":
                self.record_realtime_position()
                for point in self.trail_points:
                    pygame.draw.circle(self.window, (0, 0, 0), (point[0], point[1]), 5)

            # Draw robot
            with self.lock:
                # Change robot color based on game state and run
                if self.game_won:
                    robot_color = (0, 255, 0)  # Green when won
                elif self.game_state == "playing":
                    if self.current_run == 1:
                        robot_color = (255, 165, 0)  # Orange for normal mode
                    else:
                        robot_color = (255, 0, 255)  # Magenta for tracking mode
                else:
                    robot_color = self.icon_color  # Default green
                
                pygame.draw.circle(self.window, robot_color, (int(self.x), int(self.y)), self.icon_radius)
            
            # Draw UI elements
            self.draw_timer()
            self.draw_status_info()
            self.draw_level_info()
            self.draw_game_result()
            self.draw_instructions()
            
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
        
        # Very low stiffness for manual guidance
        self.low_stiffness = [50.0, 50.0, 50.0, 5.0, 5.0, 5.0]  # Increased X stiffness to prevent drift
        
        # Thread control flag
        self.impedance_active = False
        
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

    def generate_spline_waypoints_custom(self, start_pose, end_pose, control_points_list, spline_type='spline', num_points=40):
        """Generate spline waypoints using custom control points and type"""
        
        # Build control points array exactly like the reference code
        control_points = np.array([
            [start_pose.position.x, start_pose.position.y, start_pose.position.z],
            *control_points_list[1:-1],  # Use middle control points from the list
            [end_pose.position.x, end_pose.position.y, end_pose.position.z]
        ])
        
        rospy.loginfo(f"Control points array: {control_points}")

        if spline_type == 'spline':
            # Use cubic spline interpolation (smooth curves) - exactly like reference code
            t = np.linspace(0, 1, len(control_points))
            t_spline = np.linspace(0, 1, num_points)
            x_spline = CubicSpline(t, control_points[:, 0])(t_spline)
            y_spline = CubicSpline(t, control_points[:, 1])(t_spline)
            z_spline = CubicSpline(t, control_points[:, 2])(t_spline)

            t_spline_display = np.linspace(0, 1, 1000)
            y_spline_display = CubicSpline(t, control_points[:, 1])(t_spline_display)
            z_spline_display = CubicSpline(t, control_points[:, 2])(t_spline_display)

            # Generate waypoints exactly like reference code
            waypoints = [Pose(position=Point(x=x, y=y, z=z), orientation=start_pose.orientation)
                              for x, y, z in zip(x_spline, y_spline, z_spline)]

        else:  # 'linear' type
            # Use direct linear interpolation between points (straight line segments)
            x_path = control_points[:, 0]
            y_path = control_points[:, 1] 
            z_path = control_points[:, 2]

            waypoints = [Pose(position=Point(x=x, y=y, z=z), orientation=start_pose.orientation)
                              for x, y, z in zip(x_path, y_path, z_path)]

            # Interpolation for display - create smooth lines between waypoints
            y_path_display = []
            z_path_display = []
            
            for i in range(len(y_path) - 1):
                segment_points = 100  # Points per segment for display
                y_interpolated = np.linspace(y_path[i], y_path[i+1], segment_points, False)
                z_interpolated = np.linspace(z_path[i], z_path[i+1], segment_points, False)
                
                y_path_display.extend(y_interpolated)
                z_path_display.extend(z_interpolated)

            y_spline_display = y_path_display
            z_spline_display = z_path_display

        display_waypoints = list(zip(y_spline_display, z_spline_display))
        
        rospy.loginfo(f"Generated {len(waypoints)} waypoints")
        rospy.loginfo(f"First waypoint: x={waypoints[0].position.x:.3f}, y={waypoints[0].position.y:.3f}, z={waypoints[0].position.z:.3f}")
        rospy.loginfo(f"Last waypoint: x={waypoints[-1].position.x:.3f}, y={waypoints[-1].position.y:.3f}, z={waypoints[-1].position.z:.3f}")
        rospy.loginfo(f"Start pose: x={start_pose.position.x:.3f}, y={start_pose.position.y:.3f}, z={start_pose.position.z:.3f}")
        rospy.loginfo(f"End pose: x={end_pose.position.x:.3f}, y={end_pose.position.y:.3f}, z={end_pose.position.z:.3f}")
        
        return control_points, waypoints, display_waypoints

    def enable_low_impedance(self):
        """Enable low impedance mode for manual guidance"""
        rospy.loginfo("Enabling low impedance mode...")
        
        # Initialize thread control flag
        self.impedance_active = True
        
        # Suppress collision avoidance
        self.collision_suppress_pub.publish(Empty())
        
        # Set very low stiffness
        low_impedance_msg = create_interaction_msg(self.low_stiffness)
        self.impedance_pub.publish(low_impedance_msg)
        
        # Keep publishing to maintain low impedance
        def maintain_low_impedance():
            rate = rospy.Rate(10)  # 10 Hz
            while not rospy.is_shutdown() and self.impedance_active:
                self.collision_suppress_pub.publish(Empty())
                low_impedance_msg = create_interaction_msg(self.low_stiffness)
                self.impedance_pub.publish(low_impedance_msg)
                rate.sleep()
        
        self.impedance_thread = threading.Thread(target=maintain_low_impedance)
        self.impedance_thread.daemon = True
        self.impedance_thread.start()
        
        rospy.loginfo("Low impedance mode enabled. Robot can be manually guided.")

    def disable_impedance_control(self):
        """Disable impedance control and return to position mode"""
        rospy.loginfo("Disabling impedance control...")
        
        # Stop the impedance maintenance thread
        self.impedance_active = False
        if hasattr(self, 'impedance_thread') and self.impedance_thread.is_alive():
            self.impedance_thread.join(timeout=1.0)
        
        position_control_msg = create_position_control_msg()
        for _ in range(15):  # Publish more times to ensure mode switch
            self.impedance_pub.publish(position_control_msg)
            rospy.sleep(0.1)
        
        # Also clear any existing trajectory to ensure clean state
        self.traj.clear_waypoints()
        rospy.loginfo("Cleared trajectory waypoints.")
        
        # Wait a bit more to ensure the control mode has fully switched
        rospy.sleep(0.5)
        rospy.loginfo("Returned to position control mode.")

    def ensure_position_control(self):
        """Ensure robot is in normal position control mode with standard motion parameters"""
        rospy.loginfo("Ensuring robot is in position control mode...")
        
        # Disable any impedance control
        self.disable_impedance_control()
        
        # Re-establish position control explicitly
        rospy.loginfo("Re-establishing position control...")
        position_control_msg = create_position_control_msg()
        for _ in range(10):
            self.impedance_pub.publish(position_control_msg)
            rospy.sleep(0.1)
        
        # Clear trajectory and reset motion interface
        self.traj.clear_waypoints()
        rospy.sleep(1.0)  # Give time for mode to fully establish
        
        rospy.loginfo("Position control mode established. Ready for normal motion.")

    def execute_low_impedance_trajectory(self, spline_waypoints):
        """
        NORMAL MODE (Run 1): Move directly to final waypoint with very low impedance and extremely slow speed.
        Fixed to prevent robot from shooting backwards when reaching target.
        """
        try:
            # Enable low impedance mode first
            self.enable_low_impedance()
            rospy.sleep(1.0)  # Allow impedance mode to activate
            
            # Only use the final waypoint from the spline
            final_pose = spline_waypoints[-1]
            
            # Set extremely slow motion parameters
            wpt_opts = MotionWaypointOptions(
                max_linear_speed=0.001,      # 1mm/s - extremely slow
                max_linear_accel=0.001,      # Very gentle acceleration
                max_rotational_speed=0.001,
                max_rotational_accel=0.001,
                max_joint_speed_ratio=0.05   # Very slow joint motion
            )
            
            # Calculate IK for final position only
            joint_angles = self.calculate_ik(final_pose)
            if joint_angles is None:
                rospy.logerr("Failed to calculate IK for final position")
                return
            
            # Create single waypoint to final position
            waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
            waypoint.set_joint_angles(joint_angles, "right_hand")
            
            # Clear any existing trajectory and add only the final waypoint
            self.traj.clear_waypoints()
            self.traj.append_waypoint(waypoint.to_msg())
            
            rospy.loginfo("NORMAL MODE: Moving directly to final position with low impedance...")
            rospy.loginfo("You can now manually guide the robot arm during motion!")
            
            # Execute the trajectory
            result = self.traj.send_trajectory()
            
            if result is None:
                rospy.logerr("Trajectory FAILED to send")
            elif result.result:
                rospy.loginfo("Successfully completed normal mode motion to final position!")
            else:
                rospy.logerr(f"Trajectory execution failed with error {result.errorId}")
            
            # CRITICAL FIX: Immediately stop any motion and clear trajectory
            rospy.loginfo("Stopping robot motion to prevent backward shooting...")
            
            # Stop the robot by clearing trajectory immediately
            self.traj.clear_waypoints()  # Clear all waypoints
            rospy.sleep(0.2)
            
            # Create a new trajectory to current position to "lock" the robot in place
            try:
                current_joint_angles = self._limb.joint_angles()
                hold_wpt_opts = MotionWaypointOptions(
                    max_linear_speed=0.001,
                    max_linear_accel=0.001,
                    max_rotational_speed=0.001,
                    max_rotational_accel=0.001,
                    max_joint_speed_ratio=0.01
                )
                hold_waypoint = MotionWaypoint(options=hold_wpt_opts.to_msg(), limb=self._limb)
                hold_waypoint.set_joint_angles(current_joint_angles, "right_hand")
                
                self.traj.clear_waypoints()
                self.traj.append_waypoint(hold_waypoint.to_msg())
                rospy.loginfo("Robot locked at current position to prevent movement")
            except Exception as e:
                rospy.logwarn(f"Could not lock robot position: {e}")
            
            # Maintain low impedance but stop any active motion
            # The robot should stay compliant but not try to move anywhere
            rospy.loginfo("Maintaining low impedance mode for manual guidance only")
            
            # DO NOT disable impedance control here - keep it active so user can still guide
            # Only disable it when the game actually ends
            
        except Exception as e:
            rospy.logerr(f"Error during normal mode trajectory: {e}")
            # Safety cleanup on error
            self.traj.stop()
            self.traj.clear_waypoints()

    def execute_tracking_trajectory(self, spline_waypoints):
        """
        TRACKING MODE (Run 2): Execute trajectory through ALL spline waypoints with low impedance.
        Fixed to prevent robot from shooting backwards when reaching target.
        """
        try:
            # Enable low impedance mode first
            self.enable_low_impedance()
            rospy.sleep(1.0)  # Allow impedance mode to activate
            
            # Set slow motion parameters for tracking
            wpt_opts = MotionWaypointOptions(
                max_linear_speed=0.02,       # 20mm/s - slow but faster than normal mode
                max_linear_accel=0.02,       # Gentle acceleration
                max_rotational_speed=0.02,
                max_rotational_accel=0.02,
                max_joint_speed_ratio=0.1    # Moderate joint motion speed
            )
            
            # Clear any existing trajectory
            self.traj.clear_waypoints()
            
            # Add ALL spline waypoints to the trajectory
            rospy.loginfo(f"TRACKING MODE: Adding {len(spline_waypoints)} waypoints to trajectory...")
            
            waypoints_added = 0
            for i, pose in enumerate(spline_waypoints):
                # Calculate IK for each waypoint
                joint_angles = self.calculate_ik(pose)
                if joint_angles is None:
                    rospy.logerr(f"Failed to calculate IK for waypoint {i}")
                    continue
                
                # Create waypoint
                waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
                waypoint.set_joint_angles(joint_angles, "right_hand")
                
                # Add to trajectory
                self.traj.append_waypoint(waypoint.to_msg())
                waypoints_added += 1
            
            rospy.loginfo(f"TRACKING MODE: Successfully added {waypoints_added} waypoints to trajectory")
            rospy.loginfo("TRACKING MODE: Robot will now pull you through ALL spline points!")
            rospy.loginfo("You can still guide the robot, but it will follow the complete path!")
            
            # Execute the full trajectory
            result = self.traj.send_trajectory()
            
            if result is None:
                rospy.logerr("Tracking trajectory FAILED to send")
            elif result.result:
                rospy.loginfo("Successfully completed tracking mode through all spline points!")
            else:
                rospy.logerr(f"Tracking trajectory execution failed with error {result.errorId}")
            
            # CRITICAL FIX: Immediately stop any motion and clear trajectory
            rospy.loginfo("Stopping robot motion to prevent backward shooting...")
            
            # Stop the robot by clearing trajectory immediately
            self.traj.clear_waypoints()  # Clear all waypoints
            rospy.sleep(0.2)
            
            # Create a new trajectory to current position to "lock" the robot in place
            try:
                current_joint_angles = self._limb.joint_angles()
                hold_wpt_opts = MotionWaypointOptions(
                    max_linear_speed=0.001,
                    max_linear_accel=0.001,
                    max_rotational_speed=0.001,
                    max_rotational_accel=0.001,
                    max_joint_speed_ratio=0.01
                )
                hold_waypoint = MotionWaypoint(options=hold_wpt_opts.to_msg(), limb=self._limb)
                hold_waypoint.set_joint_angles(current_joint_angles, "right_hand")
                
                self.traj.clear_waypoints()
                self.traj.append_waypoint(hold_waypoint.to_msg())
                rospy.loginfo("Robot locked at current position to prevent movement")
            except Exception as e:
                rospy.logwarn(f"Could not lock robot position: {e}")
            
            # Maintain low impedance but stop any active motion
            rospy.loginfo("Maintaining low impedance mode for manual guidance only")
                
        except Exception as e:
            rospy.logerr(f"Error during tracking trajectory: {e}")
            # Safety cleanup on error
            self.traj.stop()
            self.traj.clear_waypoints()

    def emergency_stop(self):
        """Emergency stop method to immediately halt all robot motion"""
        rospy.loginfo("EMERGENCY STOP INITIATED")
        
        # Clear all waypoints multiple times
        for i in range(3):
            try:
                self.traj.clear_waypoints()
                rospy.sleep(0.1)
            except Exception as e:
                rospy.logwarn(f"Could not clear waypoints: {e}")
        
        # Try to lock robot at current position
        try:
            current_joint_angles = self._limb.joint_angles()
            hold_wpt_opts = MotionWaypointOptions(
                max_linear_speed=0.001,
                max_linear_accel=0.001,
                max_rotational_speed=0.001,
                max_rotational_accel=0.001,
                max_joint_speed_ratio=0.01
            )
            hold_waypoint = MotionWaypoint(options=hold_wpt_opts.to_msg(), limb=self._limb)
            hold_waypoint.set_joint_angles(current_joint_angles, "right_hand")
            
            self.traj.clear_waypoints()
            self.traj.append_waypoint(hold_waypoint.to_msg())
            rospy.loginfo("Robot locked at current position")
        except Exception as e:
            rospy.logwarn(f"Could not lock robot position: {e}")
        
        # Disable impedance and return to position control
        try:
            self.disable_impedance_control()
        except Exception as e:
            rospy.logwarn(f"Could not disable impedance control: {e}")
            
        rospy.loginfo("Emergency stop completed")


if __name__ == '__main__':
    try:
        # Get the current level and run from command line or environment
        current_level = get_current_level()
        current_run = get_current_run()
        
        # Validate level index
        if current_level >= NUM_LEVELS:
            rospy.logerr(f"Invalid level {current_level}. Maximum is {NUM_LEVELS - 1}")
            sys.exit(1)
        
        # Validate run number
        if current_run not in [1, 2]:
            rospy.logerr(f"Invalid run {current_run}. Must be 1 (Normal) or 2 (Tracking)")
            sys.exit(1)
        
        run_type = "Normal Mode" if current_run == 1 else "Tracking Mode"
        rospy.loginfo(f"Starting Sawyer Challenge at Level {current_level + 1}, {run_type}")
        
        # Create visualizer for the specific level and run
        sawyerVisualizer = SawyerVisualizer(current_level, current_run)
        sawyerVisualizer.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt detected. Exiting...")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt detected. Exiting...")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
        sys.exit(1)