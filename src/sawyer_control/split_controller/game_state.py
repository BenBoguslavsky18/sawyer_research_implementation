#!/usr/bin/env python

"""
Game state management for Sawyer Robot Challenge
Handles game logic, timing, success detection, and level progression
"""

import rospy
import time
import threading
import numpy as np
import os
import sys

from config import GAME_DURATION, NUM_LEVELS, SPLINE_CHALLENGES, SUCCESS_RADIUS, AUTO_CLOSE_DELAY


class GameState:
    """Manages the current state of the game including timing, success detection, and progression"""
    
    def __init__(self, level_index=0, run_number=1):
        # Level and run tracking
        self.current_level = level_index
        self.current_run = run_number  # 1 = normal, 2 = tracking
        self.total_levels = NUM_LEVELS
        self.current_challenge = SPLINE_CHALLENGES[level_index]
        
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
        self.success_radius = SUCCESS_RADIUS
        self.min_distance_to_target = float('inf')
        self.success_achieved = False
        self.game_won = False
        self.game_lost = False
        
        # Game result tracking
        self.final_distance = None
        self.game_completion_time = None
        self.show_results_timer = None
        self.auto_close_delay = AUTO_CLOSE_DELAY
        self.restart_next_run = False
        
        # Target position (will be set by caller)
        self.final_target_pixel = None
        
        rospy.loginfo(f"Game state initialized for {self.current_challenge['name']} - Run {run_number}")
    
    def set_target_position(self, target_pixel):
        """Set the target position in pixel coordinates"""
        self.final_target_pixel = target_pixel
    
    def start_moving_to_start(self):
        """Set game state to moving to start position"""
        self.game_state = "moving_to_start"
        self.spacebar_disabled = True
    
    def ready_to_play(self):
        """Set game state to ready to play"""
        self.robot_ready_to_move = True
        self.game_state = "ready_to_play"
        self.spacebar_disabled = False
        run_type = "Normal Mode" if self.current_run == 1 else "Tracking Mode"
        rospy.loginfo(f"Ready to start {run_type}!")
    
    def failed_to_start(self):
        """Handle failure to reach start position"""
        rospy.logerr("Failed to reach start position. Please try again.")
        self.game_state = "waiting_to_start"
        self.spacebar_disabled = False
    
    def start_game(self):
        """Start the game - initialize timer and set playing state"""
        if self.game_state == "ready_to_play":
            self.game_state = "playing"
            self.success_achieved = False
            self.game_won = False
            self.game_lost = False
            self.min_distance_to_target = float('inf')
            
            self.game_timer_active = True
            self._start_game_timer()
            return True
        return False
    
    def _start_game_timer(self):
        """Start the 30-second game timer in a separate thread"""
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
    
    def check_success(self, robot_x, robot_y):
        """
        Check if player has reached the target
        
        Args:
            robot_x, robot_y: Current robot position in pixels
            
        Returns:
            True if success achieved, False otherwise
        """
        if self.game_state == "playing" and not self.game_won and self.final_target_pixel:
            distance = np.sqrt((robot_x - self.final_target_pixel[0])**2 + 
                             (robot_y - self.final_target_pixel[1])**2)
            self.min_distance_to_target = min(self.min_distance_to_target, distance)
            
            if distance <= self.success_radius:
                self.game_won = True
                self.success_achieved = True
                self.game_completion_time = time.time() - self.game_start_time
                run_type = "Normal Mode" if self.current_run == 1 else "Tracking Mode"
                rospy.loginfo(f"=== {self.current_challenge['name']} - {run_type} COMPLETED in {self.game_completion_time:.2f} seconds ===")
                self.end_game()
                return True
        return False
    
    def get_current_distance_to_target(self, robot_x, robot_y):
        """Get current distance to target in pixels"""
        if self.final_target_pixel:
            return np.sqrt((robot_x - self.final_target_pixel[0])**2 + 
                          (robot_y - self.final_target_pixel[1])**2)
        return float('inf')
    
    def end_game(self):
        """End the game and provide final results"""
        if self.game_state == "playing":
            self.game_state = "game_over"
            self.game_timer_active = False
            
            # Calculate final distance if we have target position
            if self.final_target_pixel:
                # Note: This will be updated with actual robot position by caller
                pass
            
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
                # CHANGE: Auto-continue even after failure
                rospy.loginfo("Auto-continuing to next challenge...")
                
                # Check if this is the final run
                if self.current_level >= self.total_levels - 1 and self.current_run == 2:
                    rospy.loginfo("🎉 ALL LEVELS AND RUNS ATTEMPTED! 🎉")
                elif self.current_run == 1:
                    rospy.loginfo(f"Moving to tracking mode for level {self.current_level + 1}...")
                    self.restart_next_run = True
                else:
                    rospy.loginfo(f"Moving to level {self.current_level + 2}...")
                    self.restart_next_run = True
            
            if self.final_distance:
                rospy.loginfo(f"Final distance to target: {self.final_distance:.1f} pixels")
            rospy.loginfo(f"Closest approach: {self.min_distance_to_target:.1f} pixels")
            
            # Start auto-close timer
            self.show_results_timer = time.time()
    
    def set_final_distance(self, robot_x, robot_y):
        """Set the final distance when game ends"""
        if self.final_target_pixel:
            self.final_distance = np.sqrt((robot_x - self.final_target_pixel[0])**2 + 
                                        (robot_y - self.final_target_pixel[1])**2)
    
    def should_auto_close(self):
        """Check if the game should auto-close and restart"""
        return (self.game_state == "game_over" and 
                self.show_results_timer and 
                time.time() - self.show_results_timer >= self.auto_close_delay)
    
    def get_auto_close_remaining_time(self):
        """Get remaining time before auto-close"""
        if self.show_results_timer:
            return max(0, self.auto_close_delay - (time.time() - self.show_results_timer))
        return 0
    
    def is_final_completion(self):
        """Check if this is the final level and run completion"""
        return self.current_level >= self.total_levels - 1 and self.current_run == 2
    
    def get_run_type_string(self):
        """Get string representation of current run type"""
        return "Normal Mode" if self.current_run == 1 else "Tracking Mode"


class LevelManager:
    """Manages level progression and process restarting"""
    
    @staticmethod
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

    @staticmethod
    def get_current_run():
        """Get the current run (1 or 2) from environment variable"""
        run_env = os.environ.get('SAWYER_RUN', '1')
        try:
            return int(run_env)
        except ValueError:
            return 1
    
    @staticmethod
    def restart_for_next_run_or_level(current_level, current_run):
        """
        Restart the program for the next run or level
        
        Args:
            current_level: Current level index
            current_run: Current run number
            
        Returns:
            True if restart was initiated, False if all levels completed
        """
        import subprocess
        
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
        
        # Try using rosrun with correct package name first
        try:
            rospy.loginfo(f"Restarting using rosrun sawyer_control main.py {next_level}")
            subprocess.Popen(['rosrun', 'sawyer_control', 'main.py', str(next_level)], env=env)
            return True
        except Exception as e:
            rospy.logwarn(f"rosrun failed ({e}), trying direct python execution...")
        
        # Fallback: Find main.py in the same directory as this file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        main_script_path = os.path.join(current_dir, 'main.py')
        
        rospy.loginfo(f"Fallback: python {main_script_path} {next_level}")
        subprocess.Popen([sys.executable, main_script_path, str(next_level)], env=env)
        
        return True
    
    @staticmethod
    def restart_current_run(current_level, current_run):
        """
        Restart the current run (for failed attempts)
        
        Args:
            current_level: Current level index
            current_run: Current run number
        """
        import subprocess
        
        rospy.loginfo(f"Restarting current run: Level {current_level + 1}, Run {current_run}")
        
        env = os.environ.copy()
        env['SAWYER_LEVEL'] = str(current_level)
        env['SAWYER_RUN'] = str(current_run)
        
        # Find main.py in the same directory as this file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        main_script_path = os.path.join(current_dir, 'main.py')
        
        try:
            # Try using rosrun (more robust for ROS packages)
            subprocess.Popen(['rosrun', 'sawyer_challenge', 'main.py', str(current_level)], env=env)
        except Exception as e:
            # Fallback to direct python execution
            rospy.logwarn(f"rosrun failed ({e}), trying direct python execution...")
            subprocess.Popen([sys.executable, main_script_path, str(current_level)], env=env)