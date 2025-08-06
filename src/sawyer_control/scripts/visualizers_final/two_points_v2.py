#!/usr/bin/env python

#  Sawyer path visualizer with 30-second time limit and low impedance motion

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

# Game timing constants
GAME_DURATION = 30.0  # 30 seconds


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
        pygame.display.set_caption("Sawyer 30-Second Challenge")
        
        # Initialize font for timer display
        pygame.font.init()
        self.font = pygame.font.Font(None, 72)  # Large font for timer
        self.small_font = pygame.font.Font(None, 36)  # Smaller font for instructions
        self.big_font = pygame.font.Font(None, 120)  # Extra large font for WIN/LOSE messages

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
        """Move robot to start position and prepare for game"""
        rospy.loginfo("Moving to start position...")
        self.game_state = "moving_to_start"
        
        def move_and_signal():
            self.ik_motion.move_to_pose(start_pose)
            if not rospy.is_shutdown():
                self.robot_ready_to_move = True
                self.game_state = "ready_to_play"
                rospy.loginfo("Reached start position. Ready to start the 30-second challenge!")
        
        self.start_position_thread = threading.Thread(target=move_and_signal)
        self.start_position_thread.start()

    def start_game_timer(self):
        """Start the 30-second game timer"""
        def timer_countdown():
            rospy.loginfo("=== 30-SECOND CHALLENGE STARTED ===")
            self.game_start_time = time.time()
            self.remaining_time = GAME_DURATION
            
            while self.remaining_time > 0 and not self.game_won and not rospy.is_shutdown():
                time.sleep(0.1)  # Update every 100ms
                elapsed = time.time() - self.game_start_time
                self.remaining_time = max(0, GAME_DURATION - elapsed)
            
            # Time's up or game won
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
            self.trail_points = []  # Clear any previous trail
            self.success_achieved = False
            self.game_won = False
            self.game_lost = False
            self.min_distance_to_target = float('inf')
            
            # Start the 30-second countdown
            self.game_timer_active = True
            self.start_game_timer()
            
            # Start low impedance trajectory (robot moves slowly toward target but can be guided)
            rospy.loginfo("Starting low impedance trajectory and 30-second timer...")
            def start_trajectory():
                self.ik_motion.execute_low_impedance_trajectory(self.spline_waypoints)
            
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
                rospy.loginfo(f"=== SUCCESS! Game completed in {self.game_completion_time:.2f} seconds ===")
                self.end_game()

    def end_game(self):
        """End the game and provide final results"""
        if self.game_state == "playing":
            self.game_state = "game_over"
            self.game_timer_active = False
            
            # Calculate final distance
            self.final_distance = np.sqrt((self.x - self.final_target_pixel[0])**2 + 
                                        (self.y - self.final_target_pixel[1])**2)
            
            # Stop trajectory execution by clearing waypoints
            rospy.loginfo("Stopping robot trajectory...")
            self.ik_motion.traj.clear_waypoints()
            
            # Disable impedance control
            rospy.loginfo("Disabling impedance control...")
            self.ik_motion.disable_impedance_control()
            
            # Print final results
            rospy.loginfo("=== FINAL RESULTS ===")
            if self.game_won:
                rospy.loginfo(f"✅ SUCCESS! Completed in {self.game_completion_time:.2f} seconds")
            else:
                rospy.loginfo("❌ FAILED! Time ran out.")
            
            rospy.loginfo(f"Final distance to target: {self.final_distance:.1f} pixels")
            rospy.loginfo(f"Closest approach: {self.min_distance_to_target:.1f} pixels")
            rospy.loginfo(f"Success radius: {self.success_radius} pixels")
            
            # Start auto-close timer
            self.show_results_timer = time.time()
            rospy.loginfo(f"Game window will close automatically in {self.auto_close_delay} seconds...")

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
            # Format remaining time as MM:SS
            minutes = int(self.remaining_time // 60)
            seconds = int(self.remaining_time % 60)
            time_text = f"{minutes:02d}:{seconds:02d}"
            
            # Color changes based on remaining time
            if self.remaining_time <= 5:
                color = (255, 0, 0)  # Red for last 5 seconds
            elif self.remaining_time <= 10:
                color = (255, 255, 0)  # Yellow for last 10 seconds
            else:
                color = (255, 255, 255)  # White otherwise
            
            timer_surface = self.font.render(time_text, True, color)
            timer_rect = timer_surface.get_rect()
            timer_rect.topright = (1900, 20)
            
            # Semi-transparent background
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

    def draw_game_result(self):
        """Draw large WIN/LOSE message in center of screen"""
        if self.game_state == "game_over" or (self.game_state == "playing" and (self.game_won or self.game_lost)):
            if self.game_won:
                result_text = "YOU WIN!"
                text_color = (0, 255, 0)  # Bright green
                completion_text = f"Time: {self.game_completion_time:.1f}s"
            elif self.game_lost:
                result_text = "TIME'S UP!"
                text_color = (255, 0, 0)  # Bright red  
                completion_text = f"Distance: {self.final_distance:.1f}px"
            else:
                return
            
            # Draw main result text
            result_surface = self.big_font.render(result_text, True, text_color)
            result_rect = result_surface.get_rect()
            result_rect.center = (self.window.get_width() // 2, self.window.get_height() // 2 - 50)
            
            # Draw semi-transparent background for result
            background_width = result_rect.width + 40
            background_height = 120
            background_rect = pygame.Rect(
                result_rect.centerx - background_width // 2,
                result_rect.centery - background_height // 2,
                background_width, background_height
            )
            overlay = pygame.Surface((background_width, background_height))
            overlay.set_alpha(180)  # More opaque for visibility
            overlay.fill((0, 0, 0))  # Black background
            self.window.blit(overlay, (background_rect.left, background_rect.top))
            
            # Draw result text
            self.window.blit(result_surface, result_rect)
            
            # Draw completion details below
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
        
        if self.game_state == "waiting_to_start":
            instruction_text = "Press SPACEBAR to move robot to start position"
        elif self.game_state == "moving_to_start":
            instruction_text = "Moving to start position... Please wait"
        elif self.game_state == "ready_to_play":
            instruction_text = "Press SPACEBAR to start the 30-second challenge!"
        elif self.game_state == "playing":
            if self.game_won:
                instruction_text = "SUCCESS! You reached the target!"
            elif self.game_lost:
                instruction_text = "TIME'S UP! Game Over."
            else:
                instruction_text = "GUIDE THE ROBOT TO THE TARGET! Time is running out!"
        elif self.game_state == "game_over":
            if self.game_won:
                instruction_text = f"CONGRATULATIONS! Completed in {self.game_completion_time:.1f}s"
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
            # Check if we should auto-close after showing results
            if (self.game_state == "game_over" and self.game_won and 
                self.show_results_timer and 
                time.time() - self.show_results_timer >= self.auto_close_delay):
                rospy.loginfo("Auto-closing game window after showing results...")
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
                            # Restart the game
                            self.game_state = "waiting_to_start"
                            self.spacebar_disabled = False
                            self.robot_ready_to_move = False

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
                # Change robot color based on game state
                if self.game_won:
                    robot_color = (0, 255, 0)  # Green when won
                elif self.game_state == "playing":
                    robot_color = (255, 165, 0)  # Orange when playing
                else:
                    robot_color = self.icon_color  # Default green
                
                pygame.draw.circle(self.window, robot_color, (int(self.x), int(self.y)), self.icon_radius)
            
            # Draw UI elements
            self.draw_timer()
            self.draw_status_info()
            self.draw_game_result()  # Draw WIN/LOSE message
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
        self.low_stiffness = [50.0, 2.0, 2.0, 5.0, 5.0, 5.0]  # Increased X stiffness to prevent drift
        
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

    def enable_low_impedance(self):
        """Enable low impedance mode for manual guidance"""
        rospy.loginfo("Enabling low impedance mode...")
        
        # Suppress collision avoidance
        self.collision_suppress_pub.publish(Empty())
        
        # Set very low stiffness
        low_impedance_msg = create_interaction_msg(self.low_stiffness)
        self.impedance_pub.publish(low_impedance_msg)
        
        # Keep publishing to maintain low impedance
        def maintain_low_impedance():
            rate = rospy.Rate(10)  # 10 Hz
            while not rospy.is_shutdown():
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
        position_control_msg = create_position_control_msg()
        for _ in range(10):  # Publish multiple times to ensure mode switch
            self.impedance_pub.publish(position_control_msg)
            rospy.sleep(0.1)
        rospy.loginfo("Returned to position control mode.")

    def execute_low_impedance_trajectory(self, spline_waypoints):
        """
        Move directly to final waypoint with very low impedance and extremely slow speed.
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
            
            rospy.loginfo("Moving directly to final position with low impedance...")
            rospy.loginfo("You can now manually guide the robot arm during motion!")
            
            # Execute the trajectory
            result = self.traj.send_trajectory()
            
            if result is None:
                rospy.logerr("Trajectory FAILED to send")
            elif result.result:
                rospy.loginfo("Successfully completed low impedance motion to final position!")
            else:
                rospy.logerr(f"Trajectory execution failed with error {result.errorId}")
            
        except Exception as e:
            rospy.logerr(f"Error during low impedance trajectory: {e}")


if __name__ == '__main__':
    try:
        sawyerVisualizer = SawyerVisualizer()
        sawyerVisualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt detected. Exiting...")