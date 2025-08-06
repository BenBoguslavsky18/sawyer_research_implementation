#!/usr/bin/env python

#  Sawyer path visualizer with sequential trajectory execution

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
        rospy.init_node('sequential_trajectory_tracer', anonymous=True)

        pygame.init()
        self.window = pygame.display.set_mode((1920, 1420))
        self.background_color = (2, 114, 212)
        pygame.display.set_caption("Sawyer Sequential Trajectory Tracer")
        
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
        
        # Calculate waypoint positions in pixels for visualization
        self.waypoint_pixels = []
        for waypoint in self.spline_waypoints:
            target_x_pixel = self.convert_to_pixels_x([waypoint.position.y], -0.3, 0.5, 200, 1720)[0]
            target_y_pixel = self.convert_to_pixels_y([waypoint.position.z], 0.1, 0.5, 200, 1000)[0]
            self.waypoint_pixels.append((int(target_x_pixel), int(target_y_pixel)))

        self.icon_color = (9, 179, 54)
        self.icon_radius = 30
        self.x = 800
        self.y = 1000
        self.current_position = None  # Store actual robot position
        self.position_initialized = True
        self.scale_x = 1900
        self.scale_y = 1950
        self.lock = threading.Lock()

        self.subscriber = rospy.Subscriber("/robot/limb/right/endpoint_state",
                                           EndpointState,
                                           self.position_callback)

        # Sequential trajectory variables
        self.robot_motion_active = False
        self.robot_ready_to_move = False
        self.motion_thread = None
        self.spacebar_disabled = False
        self.trajectory_complete = False
        self.current_waypoint_index = 0
        self.completed_waypoints = []
        self.trajectory_in_progress = False
        
        # Multi-trajectory variables with manual intervention support
        self.current_trajectory_thread = None
        self.last_target_waypoint_index = -1
        self.manual_intervention_detected = False
        self.low_impedance_active = False
        
        # Timer variables
        self.timer_started = False
        self.start_time = None
        self.elapsed_time = 0.0
        
        # Success detection variables
        self.success_radius = 50  # pixels - radius for success detection
        self.waypoint_tolerance = 0.05  # meters - 3D distance tolerance for waypoint completion
        self.success_achieved = False

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
            # Store actual robot position
            self.current_position = msg.pose.position
            
            # Update screen coordinates for visualization
            screen_x = (msg.pose.position.y * self.scale_x) + 770
            screen_y = (-msg.pose.position.z * self.scale_y) + 1180
            self.x = max(0, min(1920 - self.icon_radius, screen_x))
            self.y = max(0, min(1080 - self.icon_radius, screen_y))

    def get_distance_to_waypoint(self, waypoint_index):
        """Calculate 3D distance from current position to a waypoint"""
        if self.current_position is None or waypoint_index >= len(self.spline_waypoints):
            return float('inf')
        
        waypoint = self.spline_waypoints[waypoint_index]
        dx = self.current_position.x - waypoint.position.x
        dy = self.current_position.y - waypoint.position.y
        dz = self.current_position.z - waypoint.position.z
        
        return np.sqrt(dx*dx + dy*dy + dz*dz)

    def find_next_waypoint_index(self):
        """Find the next waypoint to target based on current position and spline progress"""
        if self.current_position is None:
            return self.current_waypoint_index
        
        print(f"FIND_NEXT_WAYPOINT_INDEX CALLED:")
        print(f"  current_waypoint_index: {self.current_waypoint_index}")
        print(f"  trajectory_in_progress: {self.trajectory_in_progress}")
        print(f"  completed_waypoints: {self.completed_waypoints}")
        
        # If trajectory is in progress, don't change the target
        if self.trajectory_in_progress:
            print(f"  RETURNING current index {self.current_waypoint_index} (trajectory in progress)")
            return self.current_waypoint_index
        
        # Start searching from the current waypoint index, but allow for manual intervention
        # that might have moved us to a different part of the spline
        min_distance = float('inf')
        closest_reachable_index = self.current_waypoint_index
        
        # Check all waypoints from current index onwards
        for i in range(self.current_waypoint_index, len(self.spline_waypoints)):
            distance = self.get_distance_to_waypoint(i)
            print(f"  waypoint {i+1}: distance = {distance:.3f}m")
            
            # If we're already close to this waypoint, mark it as completed and continue
            if distance <= self.waypoint_tolerance:
                if i not in self.completed_waypoints:
                    print(f"  MARKING waypoint {i+1} as completed (close enough)")
                    self.mark_waypoint_completed(i)
                continue
            else:
                # This is the next waypoint we need to reach
                print(f"  RETURNING waypoint {i+1} as next target")
                return i
        
        # Also check if manual intervention moved us closer to a later waypoint
        # This handles cases where user manually guides the arm ahead in the sequence
        for i in range(len(self.spline_waypoints)):
            if i in self.completed_waypoints:
                continue
                
            distance = self.get_distance_to_waypoint(i)
            if distance < min_distance:
                min_distance = distance
                closest_reachable_index = i
        
        print(f"  RETURNING closest waypoint {closest_reachable_index+1}")
        return closest_reachable_index

    def detect_manual_intervention(self):
        """Detect if the robot has been manually moved significantly"""
        if self.current_position is None or not self.trajectory_in_progress:
            return False
            
        # Check if the robot is significantly closer to a different waypoint
        # than the one it's currently targeting
        current_target_distance = self.get_distance_to_waypoint(self.current_waypoint_index)
        
        for i, waypoint in enumerate(self.spline_waypoints):
            if i == self.current_waypoint_index or i in self.completed_waypoints:
                continue
                
            distance_to_other = self.get_distance_to_waypoint(i)
            
            # If we're significantly closer to another waypoint, manual intervention likely occurred
            if distance_to_other < current_target_distance * 0.7:  # 30% closer threshold
                rospy.loginfo(f"Manual intervention detected: closer to waypoint {i+1} than target {self.current_waypoint_index+1}")
                return True
                
        return False

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
                    rospy.loginfo("Reached start position. Ready for sequential trajectory motion.")
            
            self.wait_thread = threading.Thread(target=wait_for_move_completion)
            self.wait_thread.start()

    def record_realtime_position(self):
        self.trail_points.append((int(self.x), int(self.y)))

    def start_robot_motion(self):
        if self.robot_ready_to_move and not self.robot_motion_active:
            self.robot_motion_active = True
            self.current_waypoint_index = 0
            # Start the timer
            self.timer_started = True
            self.start_time = time.time()
            rospy.loginfo("Starting sequential trajectory motion... Timer started!")
            
            def motion_wrapper():
                self.ik_motion.execute_sequential_trajectory_motion(self)
                # Mark trajectory as complete
                self.trajectory_complete = True
                self.evaluate_final_result()
                rospy.loginfo("All trajectories complete. Stopping visualization...")
            
            self.motion_thread = threading.Thread(target=motion_wrapper)
            self.motion_thread.start()

    def mark_waypoint_completed(self, waypoint_index):
        """Mark a waypoint as completed"""
        if waypoint_index not in self.completed_waypoints:
            self.completed_waypoints.append(waypoint_index)
            rospy.loginfo(f"Completed waypoint {waypoint_index + 1}/{len(self.spline_waypoints)}")
            
            # Check if all waypoints are completed
            if len(self.completed_waypoints) >= len(self.spline_waypoints):
                self.success_achieved = True
                self.trajectory_complete = True
                rospy.loginfo("SUCCESS! All waypoints completed!")

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

    def draw_waypoint_indicators(self):
        """Draw all waypoint positions and their status"""
        for i, waypoint_pixel in enumerate(self.waypoint_pixels):
            if i in self.completed_waypoints:
                # Completed waypoints - green filled circle
                pygame.draw.circle(self.window, (0, 255, 0), waypoint_pixel, 15)
                pygame.draw.circle(self.window, (0, 150, 0), waypoint_pixel, 15, 3)
            elif i == self.current_waypoint_index and self.robot_motion_active:
                # Current target - yellow with pulsing effect
                pulse_radius = 15 + int(5 * np.sin(time.time() * 3))
                pygame.draw.circle(self.window, (255, 255, 0), waypoint_pixel, pulse_radius, 3)
                pygame.draw.circle(self.window, (255, 255, 0), waypoint_pixel, 8)
            else:
                # Pending waypoints - red outline
                pygame.draw.circle(self.window, (255, 0, 0), waypoint_pixel, 12, 2)
            
            # Draw waypoint number
            font_surface = pygame.font.Font(None, 24).render(str(i+1), True, (255, 255, 255))
            font_rect = font_surface.get_rect()
            font_rect.center = (waypoint_pixel[0], waypoint_pixel[1] - 25)
            self.window.blit(font_surface, font_rect)

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
        """Draw current status and waypoint progress"""
        if self.robot_motion_active:
            # Status text
            waypoint_text = f"Target Waypoint: {self.current_waypoint_index + 1}/{len(self.spline_waypoints)}"
            completed_text = f"Completed: {len(self.completed_waypoints)}/{len(self.spline_waypoints)}"
            
            if self.trajectory_in_progress:
                status_text = "Executing trajectory... (High stiffness)"
                text_color = (255, 255, 0)  # Yellow
            elif self.trajectory_complete:
                status_text = "All trajectories complete!"
                text_color = (0, 255, 0)  # Green
            elif self.low_impedance_active:
                status_text = "Low impedance - Manual guidance enabled"
                text_color = (0, 255, 255)  # Cyan
            else:
                status_text = "Ready for next trajectory"
                text_color = (255, 255, 255)  # White
            
            # Show distance to current target if available
            distance_text = ""
            intervention_text = ""
            if self.current_position and self.current_waypoint_index < len(self.spline_waypoints):
                distance = self.get_distance_to_waypoint(self.current_waypoint_index)
                distance_text = f"Distance to target: {distance:.3f}m"
                
                # Check for manual intervention
                if self.detect_manual_intervention():
                    intervention_text = "Manual intervention detected!"
            
            # Draw status lines
            y_offset = 20
            status_lines = [
                (waypoint_text, (255, 255, 255)), 
                (completed_text, (255, 255, 255)), 
                (distance_text, (255, 255, 255)),
                (intervention_text, (255, 165, 0)),  # Orange for intervention
                (status_text, text_color)
            ]
            
            for text, color in status_lines:
                if text:  # Only draw non-empty text
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
            instruction_text = "Press SPACEBAR to start sequential trajectory motion"
        elif self.robot_motion_active and not self.trajectory_complete:
            if self.low_impedance_active:
                instruction_text = "Low impedance mode - You can manually guide the arm"
            else:
                instruction_text = "Robot executing sequential trajectories automatically"
        elif self.trajectory_complete:
            instruction_text = "Motion complete!"
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

            # Update timer
            self.update_timer()

            self.window.fill(self.background_color)
            
            # Draw spline path as guide
            for i, point in enumerate(self.smooth_points):
                # Dim the path for completed sections
                alpha = 100 if i < (len(self.completed_waypoints) * len(self.smooth_points) // len(self.spline_waypoints)) else 255
                color = (255, 0, 0) if alpha == 255 else (100, 0, 0)
                pygame.draw.circle(self.window, color, (int(point[0]), int(point[1])), 3)
            
            # Draw waypoint indicators
            self.draw_waypoint_indicators()
            
            # Draw real-time trail
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


'''ROBOT MOTION WITH SEQUENTIAL TRAJECTORIES'''
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
        
        # Low stiffness for manual guidance between waypoints
        self.low_stiffness = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0]  # Low stiffness for manual guidance
        
        rospy.loginfo("IK and Sequential Trajectory Motion setup complete.")

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

    def move_to_pose(self, pose, limb_name="right_hand", speed_ratio=0.01):
        """Move to a pose using normal trajectory execution"""
        joint_angles = self.calculate_ik(pose)
        if joint_angles is None:
            rospy.logerr("Failed to move to pose: IK solution not found")
            return False
            
        wpt_opts = MotionWaypointOptions(
            max_linear_speed=0.2 * speed_ratio,
            max_linear_accel=0.2 * speed_ratio,
            max_rotational_speed=0.2 * speed_ratio,
            max_rotational_accel=0.2 * speed_ratio,
            max_joint_speed_ratio=speed_ratio
        )
        
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
        waypoint.set_joint_angles(joint_angles, limb_name)
        self.traj.clear_waypoints()
        self.traj.append_waypoint(waypoint.to_msg())
        
        result = self.traj.send_trajectory()
        if result is None:
            rospy.logerr("Failed to move to pose: Trajectory failed to send")
            return False
        elif result.result:
            rospy.loginfo("Successfully moved to pose!")
            return True
        else:
            rospy.logerr(f"Failed to move to pose with error {result.errorId}")
            return False

    def enable_low_impedance(self, visualizer):
        """Enable low impedance mode for manual guidance between waypoints"""
        print("="*50)
        print("ENTERING ENABLE_LOW_IMPEDANCE FUNCTION")
        print("="*50)
        rospy.loginfo("Enabling low impedance mode for manual guidance...")
        
        # Suppress collision avoidance
        print("Publishing collision suppression...")
        self.collision_suppress_pub.publish(Empty())
        
        # Set low stiffness
        print(f"Creating low impedance message with stiffness: {self.low_stiffness}")
        low_impedance_msg = create_interaction_msg(self.low_stiffness)
        print("Publishing low impedance message...")
        self.impedance_pub.publish(low_impedance_msg)
        
        # Update visualizer status
        print("Updating visualizer status...")
        visualizer.low_impedance_active = True
        
        print("LOW IMPEDANCE MODE SHOULD NOW BE ACTIVE!")
        print("="*50)
        rospy.loginfo("Low impedance mode enabled. Robot can be manually guided.")

    def disable_impedance_control(self, visualizer):
        """Disable impedance control and return to position mode"""
        print("="*50)
        print("ENTERING DISABLE_IMPEDANCE_CONTROL FUNCTION")
        print("="*50)
        rospy.loginfo("Disabling impedance control...")
        position_control_msg = create_position_control_msg()
        for i in range(3):  # Publish multiple times to ensure mode switch
            print(f"Publishing position control message {i+1}/3...")
            self.impedance_pub.publish(position_control_msg)
            rospy.sleep(0.1)
        
        # Update visualizer status
        print("Updating visualizer status...")
        visualizer.low_impedance_active = False
        
        print("IMPEDANCE CONTROL DISABLED!")
        print("="*50)
        rospy.loginfo("Returned to position control mode.")
    def generate_spline_waypoints(self, start_pose, end_pose, num_points=15):
        """Generate waypoints along a spline path"""
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

        # For display
        t_spline_display = np.linspace(0, 1, 1000)
        y_spline_display = CubicSpline(t, control_points[:, 1])(t_spline_display)
        z_spline_display = CubicSpline(t, control_points[:, 2])(t_spline_display)

        self.waypoints = [Pose(position=Point(x=x, y=y, z=z), orientation=start_pose.orientation)
                          for x, y, z in zip(x_spline, y_spline, z_spline)]

        display_waypoints = list(zip(y_spline_display, z_spline_display))
        return control_points, self.waypoints, display_waypoints

    def execute_trajectory_to_waypoint(self, target_waypoint, waypoint_index, visualizer):
        """Execute a single trajectory to a waypoint in a separate thread"""
        try:
            print(f"STARTING TRAJECTORY TO WAYPOINT {waypoint_index + 1}")
            print(f"Current low_impedance_active status: {visualizer.low_impedance_active}")
            
            # Disable low impedance mode for precise trajectory execution
            if visualizer.low_impedance_active:
                print("LOW IMPEDANCE IS ACTIVE - DISABLING IT NOW")
                self.disable_impedance_control(visualizer)
                rospy.sleep(0.5)  # Allow mode switch to complete
            else:
                print("Low impedance was not active, proceeding with trajectory")
            
            rospy.loginfo(f"Starting trajectory to waypoint {waypoint_index + 1}")
            success = self.move_to_pose(target_waypoint, speed_ratio=0.3)
            
            print(f"TRAJECTORY EXECUTION RESULT: {success}")
            
            if success:
                # Check if we actually reached the waypoint
                distance = visualizer.get_distance_to_waypoint(waypoint_index)
                print(f"Distance to waypoint after trajectory: {distance:.3f}m (tolerance: {visualizer.waypoint_tolerance}m)")
                if distance <= visualizer.waypoint_tolerance:
                    visualizer.mark_waypoint_completed(waypoint_index)
                    rospy.loginfo(f"Successfully reached waypoint {waypoint_index + 1}")
                else:
                    rospy.logwarn(f"Trajectory completed but waypoint {waypoint_index + 1} not reached (distance: {distance:.3f}m)")
            else:
                rospy.logerr(f"Failed to reach waypoint {waypoint_index + 1}")
                
        except Exception as e:
            rospy.logerr(f"Error executing trajectory to waypoint {waypoint_index + 1}: {e}")
        finally:
            print(f"TRAJECTORY TO WAYPOINT {waypoint_index + 1} FINISHED - SETTING trajectory_in_progress = False")
            visualizer.trajectory_in_progress = False

    def execute_sequential_trajectory_motion(self, visualizer):
        """
        Execute trajectories sequentially with manual intervention capability
        """
        try:
            rospy.loginfo(f"Starting sequential trajectory motion with {len(self.waypoints)} waypoints")
            
            current_trajectory_thread = None
            last_target_waypoint_index = -1
            
            while not rospy.is_shutdown() and not visualizer.trajectory_complete:
                # Find the next waypoint to target based on current position
                next_waypoint_index = visualizer.find_next_waypoint_index()
                
                # Check if the target waypoint has changed (due to manual intervention)
                if next_waypoint_index != last_target_waypoint_index:
                    rospy.loginfo(f"Target waypoint changed from {last_target_waypoint_index + 1} to {next_waypoint_index + 1}")
                    
                    # Kill the previous trajectory if it's still running
                    if current_trajectory_thread and current_trajectory_thread.is_alive():
                        rospy.loginfo("Stopping previous trajectory due to waypoint change...")
                        # Stop the current trajectory using limb interface
                        try:
                            self._limb.exit_control_mode()
                            rospy.sleep(0.1)
                            # Clear any pending trajectory
                            self.traj.clear_waypoints()
                        except Exception as e:
                            rospy.logwarn(f"Error stopping trajectory: {e}")
                        
                        # Wait for thread to finish or force terminate
                        current_trajectory_thread.join(timeout=1.0)
                        if current_trajectory_thread.is_alive():
                            rospy.logwarn("Previous trajectory thread did not terminate cleanly")
                    
                    # If we're already close to all remaining waypoints, we're done
                    if next_waypoint_index >= len(self.waypoints):
                        rospy.loginfo("Reached end of trajectory sequence")
                        break
                    
                    # Update the current target
                    visualizer.current_waypoint_index = next_waypoint_index
                    target_waypoint = self.waypoints[next_waypoint_index]
                    
                    rospy.loginfo(f"Starting new trajectory to waypoint {next_waypoint_index + 1}/{len(self.waypoints)}")
                    visualizer.trajectory_in_progress = True
                    
                    # Start new trajectory thread
                    current_trajectory_thread = threading.Thread(
                        target=self.execute_trajectory_to_waypoint,
                        args=(target_waypoint, next_waypoint_index, visualizer)
                    )
                    current_trajectory_thread.daemon = True
                    current_trajectory_thread.start()
                    
                    last_target_waypoint_index = next_waypoint_index
                
                # Check trajectory status periodically
                if current_trajectory_thread and not current_trajectory_thread.is_alive():
                    print("="*60)
                    print("TRAJECTORY THREAD HAS COMPLETED!")
                    print(f"visualizer.trajectory_complete: {visualizer.trajectory_complete}")
                    print(f"visualizer.low_impedance_active: {visualizer.low_impedance_active}")
                    print(f"visualizer.trajectory_in_progress: {visualizer.trajectory_in_progress}")
                    print("="*60)
                    
                    # Trajectory completed, enable low impedance for manual guidance
                    if not visualizer.trajectory_complete and not visualizer.low_impedance_active:
                        print("CONDITIONS MET - CALLING ENABLE_LOW_IMPEDANCE!")
                        rospy.loginfo("Trajectory completed, enabling low impedance for manual guidance...")
                        self.enable_low_impedance(visualizer)
                    else:
                        print("CONDITIONS NOT MET FOR ENABLING LOW IMPEDANCE:")
                        print(f"  trajectory_complete: {visualizer.trajectory_complete}")
                        print(f"  low_impedance_active: {visualizer.low_impedance_active}")
                    
                    # Wait a moment before checking for next waypoint
                    rospy.sleep(1.0)
                    
                    # Reset trajectory status
                    if not visualizer.trajectory_in_progress:
                        last_target_waypoint_index = -1  # Force re-evaluation of next waypoint
                
                # Sleep briefly to allow for manual intervention detection
                rospy.sleep(3.0)
            
            # Clean up any remaining trajectory thread
            if current_trajectory_thread and current_trajectory_thread.is_alive():
                rospy.loginfo("Waiting for final trajectory to complete...")
                current_trajectory_thread.join()
            
            # Disable impedance control when motion is complete
            if visualizer.low_impedance_active:
                self.disable_impedance_control(visualizer)
            
            rospy.loginfo("Sequential trajectory motion completed!")
            
        except Exception as e:
            rospy.logerr(f"Error during sequential trajectory motion: {e}")


if __name__ == '__main__':
    try:
        sawyerVisualizer = SawyerVisualizer()
        sawyerVisualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt detected. Exiting...")