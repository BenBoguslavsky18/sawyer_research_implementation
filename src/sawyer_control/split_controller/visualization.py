#!/usr/bin/env python

"""
Visualization and UI for Sawyer Robot Challenge
Handles PyGame display, rendering, and user interface
"""

import pygame
import rospy
import threading
import numpy as np

from config import (BACKGROUND_COLOR, ROBOT_COLOR_DEFAULT, ROBOT_COLOR_WON, 
                   ROBOT_COLOR_NORMAL_MODE, ROBOT_COLOR_TRACKING_MODE,
                   SPLINE_PATH_COLOR, TRAIL_COLOR, TARGET_SUCCESS_COLOR, 
                   TARGET_CENTER_COLOR, ROBOT_ICON_RADIUS)


class UIRenderer:
    """Handles rendering of UI elements like timer, status, instructions"""
    
    def __init__(self):
        pygame.font.init()
        self.font = pygame.font.Font(None, 72)  # Large font for timer
        self.small_font = pygame.font.Font(None, 36)  # Smaller font for instructions
        self.big_font = pygame.font.Font(None, 120)  # Extra large font for WIN/LOSE messages
    
    def draw_timer(self, window, game_state):
        """Draw the game timer"""
        if game_state.game_timer_active and game_state.game_state == "playing":
            minutes = int(game_state.remaining_time // 60)
            seconds = int(game_state.remaining_time % 60)
            time_text = f"{minutes:02d}:{seconds:02d}"
            
            if game_state.remaining_time <= 5:
                color = (255, 0, 0)
            elif game_state.remaining_time <= 10:
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
            window.blit(overlay, (background_rect.left, background_rect.top))
            
            window.blit(timer_surface, timer_rect)

    def draw_status_info(self, window, game_state, robot_x, robot_y):
        """Draw current distance and game status"""
        if game_state.game_state == "playing" and game_state.final_target_pixel:
            current_distance = game_state.get_current_distance_to_target(robot_x, robot_y)
            
            status_text = f"Distance: {current_distance:.1f}px"
            if game_state.success_achieved:
                status_text += " - YOU WIN!"
                text_color = (0, 255, 0)
            elif current_distance <= game_state.success_radius:
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
            window.blit(overlay, (background_rect.left, background_rect.top))
            
            window.blit(status_surface, status_rect)

    def draw_level_info(self, window, game_state):
        """Draw current level and run information"""
        run_type = game_state.get_run_type_string().replace(" Mode", "")
        level_text = f"Level {game_state.current_level + 1}/{game_state.total_levels}: {game_state.current_challenge['name']} - {run_type} Mode"
        level_surface = self.small_font.render(level_text, True, (255, 255, 255))
        level_rect = level_surface.get_rect()
        level_rect.topleft = (20, 60)
        
        background_rect = pygame.Rect(level_rect.left - 10, level_rect.top - 5,
                                    level_rect.width + 20, level_rect.height + 10)
        overlay = pygame.Surface((background_rect.width, background_rect.height))
        overlay.set_alpha(128)
        overlay.fill((0, 0, 0))
        window.blit(overlay, (background_rect.left, background_rect.top))
        
        window.blit(level_surface, level_rect)

    def draw_game_result(self, window, game_state):
        """Draw large WIN/LOSE message in center of screen"""
        if game_state.game_state == "game_over" or (game_state.game_state == "playing" and (game_state.game_won or game_state.game_lost)):
            if game_state.game_won:
                if game_state.is_final_completion():
                    result_text = "ALL COMPLETE!"
                    text_color = (255, 215, 0)  # Gold for final completion
                elif game_state.current_run == 1:
                    result_text = "RUN 1 COMPLETE!"
                    text_color = (0, 255, 0)  # Green for run 1
                else:
                    result_text = "RUN 2 COMPLETE!"
                    text_color = (0, 255, 0)  # Green for run 2
                completion_text = f"Time: {game_state.game_completion_time:.1f}s"
            elif game_state.game_lost:
                result_text = "TIME'S UP!"
                text_color = (255, 0, 0)
                completion_text = f"Distance: {game_state.final_distance:.1f}px" if game_state.final_distance else "Game Over"
            else:
                return
            
            # Draw main result text
            result_surface = self.big_font.render(result_text, True, text_color)
            result_rect = result_surface.get_rect()
            result_rect.center = (window.get_width() // 2, window.get_height() // 2 - 50)
            
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
            window.blit(overlay, (background_rect.left, background_rect.top))
            
            window.blit(result_surface, result_rect)
            
            # Draw completion details
            if completion_text:
                detail_surface = self.font.render(completion_text, True, (255, 255, 255))
                detail_rect = detail_surface.get_rect()
                detail_rect.center = (window.get_width() // 2, result_rect.bottom + 30)
                
                # Background for detail text
                detail_bg_rect = pygame.Rect(detail_rect.left - 10, detail_rect.top - 5,
                                           detail_rect.width + 20, detail_rect.height + 10)
                detail_overlay = pygame.Surface((detail_bg_rect.width, detail_bg_rect.height))
                detail_overlay.set_alpha(150)
                detail_overlay.fill((0, 0, 0))
                window.blit(detail_overlay, (detail_bg_rect.left, detail_bg_rect.top))
                
                window.blit(detail_surface, detail_rect)

    def draw_instructions(self, window, game_state):
        """Draw instructions based on current game state"""
        instruction_text = ""
        run_type = game_state.get_run_type_string()
        
        if game_state.game_state == "waiting_to_start":
            instruction_text = f"Press SPACEBAR to move robot to start position ({run_type})"
        elif game_state.game_state == "moving_to_start":
            instruction_text = f"Moving to start position for {run_type}... Please wait"
        elif game_state.game_state == "ready_to_play":
            if game_state.current_run == 1:
                instruction_text = "Press SPACEBAR to start Normal Mode (low impedance to end point)"
            else:
                instruction_text = "Press SPACEBAR to start Tracking Mode (robot follows all spline points)"
        elif game_state.game_state == "playing":
            if game_state.game_won:
                instruction_text = "SUCCESS! You reached the target!"
            elif game_state.game_lost:
                instruction_text = "TIME'S UP! Game Over."
            else:
                if game_state.current_run == 1:
                    instruction_text = "NORMAL MODE: Guide robot to target! (Robot moves to end point)"
                else:
                    instruction_text = "TRACKING MODE: Guide robot to target! (Robot follows spline path)"
        elif game_state.game_state == "game_over":
            if game_state.game_won:
                if game_state.is_final_completion():
                    instruction_text = "🎉 ALL LEVELS AND RUNS COMPLETED! Window will close automatically..."
                else:
                    if game_state.show_results_timer:
                        remaining_time = game_state.get_auto_close_remaining_time()
                        if game_state.current_run == 1:
                            instruction_text = f"Starting Tracking Mode soon... ({remaining_time:.1f}s)"
                        else:
                            instruction_text = f"Next level starting soon... ({remaining_time:.1f}s)"
                    else:
                        if game_state.current_run == 1:
                            instruction_text = "Starting Tracking Mode soon..."
                        else:
                            instruction_text = "Next level starting soon..."
            else:
                # CHANGED: Show auto-continue message instead of manual restart
                if game_state.restart_next_run:
                    if game_state.is_final_completion():
                        instruction_text = "🎉 ALL LEVELS ATTEMPTED! Window will close automatically..."
                    else:
                        if game_state.show_results_timer:
                            remaining_time = game_state.get_auto_close_remaining_time()
                            if game_state.current_run == 1:
                                instruction_text = f"Auto-continuing to Tracking Mode... ({remaining_time:.1f}s)"
                            else:
                                instruction_text = f"Auto-continuing to next level... ({remaining_time:.1f}s)"
                        else:
                            if game_state.current_run == 1:
                                instruction_text = "Auto-continuing to Tracking Mode..."
                            else:
                                instruction_text = "Auto-continuing to next level..."
                else:
                    instruction_text = "Failed - Press SPACEBAR to try again"
        
        if instruction_text:
            text_surface = self.small_font.render(instruction_text, True, (255, 255, 255))
            text_rect = text_surface.get_rect()
            text_rect.centerx = window.get_width() // 2
            text_rect.bottom = window.get_height() - 20
            
            background_rect = pygame.Rect(text_rect.left - 10, text_rect.top - 5,
                                        text_rect.width + 20, text_rect.height + 10)
            overlay = pygame.Surface((background_rect.width, background_rect.height))
            overlay.set_alpha(128)
            overlay.fill((0, 0, 0))
            window.blit(overlay, (background_rect.left, background_rect.top))
            
            window.blit(text_surface, text_rect)


class SawyerVisualizer:
    """Main visualization class handling PyGame display and robot visualization"""
    
    def __init__(self, game_state, coordinate_converter):
        pygame.init()
        self.window = pygame.display.set_mode((1920, 1420))
        self.background_color = BACKGROUND_COLOR
        
        # Game state and coordinate converter
        self.game_state = game_state
        self.coord_converter = coordinate_converter
        
        # UI renderer
        self.ui_renderer = UIRenderer()
        
        # Update window caption
        run_type = game_state.get_run_type_string()
        pygame.display.set_caption(f"Sawyer Challenge - {game_state.current_challenge['name']} - {run_type}")
        
        # Robot position and display
        self.icon_radius = ROBOT_ICON_RADIUS
        self.x = 800
        self.y = 1000
        self.lock = threading.Lock()
        
        # Path visualization
        self.smooth_points = []
        self.trail_points = []
        
        rospy.loginfo(f"Visualizer initialized for {game_state.current_challenge['name']} - {run_type}")
    
    def set_path_points(self, display_waypoints):
        """Set the path points for visualization"""
        x_coords = [p[0] for p in display_waypoints]
        y_coords = [p[1] for p in display_waypoints]
        x_coords = self.coord_converter.convert_to_pixels_x(x_coords)
        y_coords = self.coord_converter.convert_to_pixels_y(y_coords)
        self.smooth_points = list(zip(x_coords, y_coords))
    
    def set_target_position(self, target_pose):
        """Set and calculate target position in pixels"""
        target_pixel = self.coord_converter.get_target_pixel_position(target_pose)
        self.game_state.set_target_position(target_pixel)
    
    def update_robot_position(self, robot_pose):
        """Update robot position from ROS message"""
        with self.lock:
            self.x, self.y = self.coord_converter.robot_pose_to_screen_coords(robot_pose)
    
    def record_trail_point(self):
        """Record current robot position for trail visualization"""
        if self.game_state.game_state == "playing":
            self.trail_points.append((int(self.x), int(self.y)))
    
    def draw_target_indicator(self):
        """Draw the target position and success radius"""
        if self.game_state.final_target_pixel:
            # Draw success radius circle
            pygame.draw.circle(self.window, TARGET_SUCCESS_COLOR, 
                             self.game_state.final_target_pixel, 
                             self.game_state.success_radius, 3)
            
            # Draw target center
            pygame.draw.circle(self.window, TARGET_CENTER_COLOR, 
                             self.game_state.final_target_pixel, 10)
            
            # Draw crosshair
            cross_size = 15
            pygame.draw.line(self.window, TARGET_CENTER_COLOR, 
                           (self.game_state.final_target_pixel[0] - cross_size, self.game_state.final_target_pixel[1]),
                           (self.game_state.final_target_pixel[0] + cross_size, self.game_state.final_target_pixel[1]), 3)
            pygame.draw.line(self.window, TARGET_CENTER_COLOR,
                           (self.game_state.final_target_pixel[0], self.game_state.final_target_pixel[1] - cross_size),
                           (self.game_state.final_target_pixel[0], self.game_state.final_target_pixel[1] + cross_size), 3)
    
    def draw_spline_path(self):
        """Draw the spline path as guide"""
        for point in self.smooth_points:
            pygame.draw.circle(self.window, SPLINE_PATH_COLOR, (int(point[0]), int(point[1])), 5)
    
    def draw_trail(self):
        """Draw the robot's movement trail"""
        for point in self.trail_points:
            pygame.draw.circle(self.window, TRAIL_COLOR, (point[0], point[1]), 5)
    
    def draw_robot(self):
        """Draw the robot icon with appropriate color"""
        with self.lock:
            # Choose robot color based on state and run
            if self.game_state.game_won:
                robot_color = ROBOT_COLOR_WON
            elif self.game_state.game_state == "playing":
                if self.game_state.current_run == 1:
                    robot_color = ROBOT_COLOR_NORMAL_MODE
                else:
                    robot_color = ROBOT_COLOR_TRACKING_MODE
            else:
                robot_color = ROBOT_COLOR_DEFAULT
            
            pygame.draw.circle(self.window, robot_color, (int(self.x), int(self.y)), self.icon_radius)
    
    def handle_pygame_events(self):
        """
        Handle PyGame events and return action to take
        
        Returns:
            String indicating action: 'quit', 'move_to_start', 'start_game', 'restart_run', or None
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return 'quit'
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    if self.game_state.game_state == "waiting_to_start":
                        return 'move_to_start'
                    elif self.game_state.game_state == "ready_to_play":
                        return 'start_game'
                    elif self.game_state.game_state == "game_over" and not self.game_state.game_won:
                        return 'restart_run'
        return None
    
    def update_and_check_success(self):
        """Update trail and check for success"""
        if self.game_state.game_state == "playing":
            self.record_trail_point()
            self.game_state.check_success(self.x, self.y)
    
    def render_frame(self):
        """Render a single frame of the visualization"""
        # Clear screen
        self.window.fill(self.background_color)
        
        # Draw path elements
        self.draw_spline_path()
        self.draw_target_indicator()
        
        # Draw trail during gameplay
        if self.game_state.game_state == "playing":
            self.draw_trail()
        
        # Draw robot
        self.draw_robot()
        
        # Draw UI elements
        self.ui_renderer.draw_timer(self.window, self.game_state)
        self.ui_renderer.draw_status_info(self.window, self.game_state, self.x, self.y)
        self.ui_renderer.draw_level_info(self.window, self.game_state)
        self.ui_renderer.draw_game_result(self.window, self.game_state)
        self.ui_renderer.draw_instructions(self.window, self.game_state)
        
        # Update display
        pygame.display.flip()
    
    def cleanup(self):
        """Clean up pygame resources"""
        pygame.quit()