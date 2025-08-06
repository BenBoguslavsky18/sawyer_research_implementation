#!/usr/bin/env python

"""
Main entry point for Sawyer Robot Challenge
Coordinates all components and manages the game loop

This is a ROS node - run with: rosrun sawyer_challenge main.py
"""

import rospy
import sys
import threading
import time
import pygame

from intera_core_msgs.msg import EndpointState

from config import NUM_LEVELS, SPLINE_CHALLENGES
from game_state import GameState, LevelManager
from robot_control import IKMotionWaypoint
from coordinate_utils import CoordinateConverter
from visualization import SawyerVisualizer


class SawyerChallengeApplication:
    """Main application class that coordinates all components"""
    
    def __init__(self, level_index=0, run_number=1):
        # Initialize ROS
        rospy.init_node('impedance_spline_tracer', anonymous=True)
        
        # Validate inputs
        if level_index >= NUM_LEVELS:
            rospy.logerr(f"Invalid level {level_index}. Maximum is {NUM_LEVELS - 1}")
            sys.exit(1)
        
        if run_number not in [1, 2]:
            rospy.logerr(f"Invalid run {run_number}. Must be 1 (Normal) or 2 (Tracking)")
            sys.exit(1)
        
        # Initialize components
        self.game_state = GameState(level_index, run_number)
        self.coord_converter = CoordinateConverter()
        self.robot_control = IKMotionWaypoint()
        self.visualizer = SawyerVisualizer(self.game_state, self.coord_converter)
        
        # Generate spline for current challenge
        self._setup_current_level()
        
        # ROS subscriber for robot position
        self.position_subscriber = rospy.Subscriber(
            "/robot/limb/right/endpoint_state",
            EndpointState,
            self._position_callback
        )
        
        run_type = self.game_state.get_run_type_string()
        rospy.loginfo(f"Sawyer Challenge initialized - {self.game_state.current_challenge['name']} - {run_type}")
    
    def _setup_current_level(self):
        """Setup the current level with spline generation and visualization"""
        challenge = self.game_state.current_challenge
        start_pose = challenge['start']
        end_pose = challenge['end']
        control_points = challenge['control_points']
        spline_type = challenge.get('type', 'spline')
        
        # Generate spline waypoints
        rospy.loginfo("Generating spline waypoints...")
        self.spline_controlpoints, self.spline_waypoints, self.display_waypoints = \
            self.robot_control.generate_spline_waypoints_custom(
                start_pose, end_pose, control_points, spline_type)
        
        # Setup visualization
        self.visualizer.set_path_points(self.display_waypoints)
        self.visualizer.set_target_position(end_pose)
        
        rospy.loginfo(f"Level setup complete: {len(self.spline_waypoints)} waypoints generated")
    
    def _position_callback(self, msg):
        """ROS callback for robot position updates"""
        self.visualizer.update_robot_position(msg.pose)
    
    def _move_to_start_position(self):
        """Move robot to start position in a separate thread"""
        def move_and_signal():
            self.game_state.start_moving_to_start()
            
            start_pose = self.game_state.current_challenge['start']
            rospy.loginfo(f"Moving to start position: x={start_pose.position.x}, y={start_pose.position.y}, z={start_pose.position.z}")
            
            # Move to start position
            success = self.robot_control.move_to_pose(start_pose)
            
            if not rospy.is_shutdown():
                if success:
                    self.game_state.ready_to_play()
                    rospy.loginfo("Successfully reached start position!")
                else:
                    self.game_state.failed_to_start()
        
        move_thread = threading.Thread(target=move_and_signal)
        move_thread.daemon = True
        move_thread.start()
    
    def _start_game_trajectory(self):
        """Start the appropriate trajectory based on run type"""
        def trajectory_execution():
            if self.game_state.current_run == 1:
                # Run 1: Normal low impedance trajectory (just to end point)
                rospy.loginfo("Starting low impedance trajectory to end point (Normal Mode)...")
                self.robot_control.execute_low_impedance_trajectory(self.spline_waypoints)
            else:
                # Run 2: Tracking trajectory (through all spline points)
                rospy.loginfo("Starting tracking trajectory through all spline points (Tracking Mode)...")
                self.robot_control.execute_tracking_trajectory(self.spline_waypoints)
        
        trajectory_thread = threading.Thread(target=trajectory_execution)
        trajectory_thread.daemon = True
        trajectory_thread.start()
    
    def _handle_game_end(self):
        """Handle game end cleanup and stopping robot motion"""
        rospy.loginfo("Game ended - stopping robot motion...")
        
        # Set final distance
        self.game_state.set_final_distance(self.visualizer.x, self.visualizer.y)
        
        # Emergency stop robot motion
        self.robot_control.emergency_stop()
        
        rospy.loginfo("Robot motion stopped and impedance control disabled.")
    
    def run(self):
        """Main application loop"""
        clock = pygame.time.Clock()
        running = True
        
        rospy.loginfo("Starting main game loop...")
        
        while running and not rospy.is_shutdown():
            # Check for auto-close condition
            if self.game_state.should_auto_close():
                if self.game_state.restart_next_run:
                    if self.game_state.is_final_completion():
                        rospy.loginfo("All levels and runs completed! Closing game...")
                        running = False
                    else:
                        # Restart for next run or level
                        rospy.loginfo("Closing pygame and restarting for next run/level...")
                        success = LevelManager.restart_for_next_run_or_level(
                            self.game_state.current_level, 
                            self.game_state.current_run
                        )
                        running = False
                else:
                    # Failed run, just close
                    running = False
            
            # Handle pygame events
            action = self.visualizer.handle_pygame_events()
            
            if action == 'quit':
                running = False
            elif action == 'move_to_start':
                if not self.game_state.spacebar_disabled:
                    self._move_to_start_position()
            elif action == 'start_game':
                if self.game_state.start_game():
                    self._start_game_trajectory()
            elif action == 'restart_run':
                # Only allow manual restart if auto-continue is disabled
                if not self.game_state.restart_next_run:
                    rospy.loginfo("Manual restart requested...")
                    LevelManager.restart_current_run(
                        self.game_state.current_level,
                        self.game_state.current_run
                    )
                    running = False
            
            # Update game state and check for success
            self.visualizer.update_and_check_success()
            
            # Handle game end if it just occurred
            if (self.game_state.game_state == "playing" and 
                (self.game_state.game_won or self.game_state.game_lost) and
                not hasattr(self, '_game_end_handled')):
                self._handle_game_end()
                self._game_end_handled = True
            
            # Render the frame
            self.visualizer.render_frame()
            
            # Control frame rate
            clock.tick(60)
        
        # Cleanup
        self.visualizer.cleanup()
        rospy.signal_shutdown("Application closing")


def main():
    """Main entry point"""
    try:
        # Get the current level and run from command line or environment
        current_level = LevelManager.get_current_level()
        current_run = LevelManager.get_current_run()
        
        run_type = "Normal Mode" if current_run == 1 else "Tracking Mode"
        rospy.loginfo(f"Starting Sawyer Challenge at Level {current_level + 1}, {run_type}")
        
        # Create and run the application
        app = SawyerChallengeApplication(current_level, current_run)
        app.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt detected. Exiting...")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt detected. Exiting...")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()