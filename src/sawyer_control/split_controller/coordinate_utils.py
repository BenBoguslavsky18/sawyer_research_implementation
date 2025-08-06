#!/usr/bin/env python

"""
Coordinate conversion utilities for Sawyer Robot Challenge
Handles conversion between robot coordinates and screen pixels
"""

import rospy
from config import COORDINATE_RANGES, SCALE_X, SCALE_Y, ROBOT_POSITION_OFFSET_X, ROBOT_POSITION_OFFSET_Y


class CoordinateConverter:
    """Handles conversion between robot coordinate system and screen pixel coordinates"""
    
    def __init__(self):
        self.ranges = COORDINATE_RANGES
        self.scale_x = SCALE_X
        self.scale_y = SCALE_Y
        self.offset_x = ROBOT_POSITION_OFFSET_X
        self.offset_y = ROBOT_POSITION_OFFSET_Y
    
    def convert_to_pixels_x(self, meter_values, m_min=None, m_max=None, p_min=None, p_max=None):
        """
        Convert robot Y coordinates (meters) to screen X coordinates (pixels)
        
        Args:
            meter_values: List of meter values to convert
            m_min, m_max: Robot coordinate range (uses defaults if None)
            p_min, p_max: Screen pixel range (uses defaults if None)
        
        Returns:
            List of pixel coordinates
        """
        if m_min is None:
            m_min = self.ranges['robot_y_min']
        if m_max is None:
            m_max = self.ranges['robot_y_max']
        if p_min is None:
            p_min = self.ranges['screen_x_min']
        if p_max is None:
            p_max = self.ranges['screen_x_max']
            
        return [p_min + ((m - m_min) / (m_max - m_min)) * (p_max - p_min) for m in meter_values]
    
    def convert_to_pixels_y(self, meter_values, m_min=None, m_max=None, p_min=None, p_max=None):
        """
        Convert robot Z coordinates (meters) to screen Y coordinates (pixels)
        Special handling for values above 0.3 meters
        
        Args:
            meter_values: List of meter values to convert
            m_min, m_max: Robot coordinate range (uses defaults if None)
            p_min, p_max: Screen pixel range (uses defaults if None)
        
        Returns:
            List of pixel coordinates
        """
        if m_min is None:
            m_min = self.ranges['robot_z_min']
        if m_max is None:
            m_max = self.ranges['robot_z_max']
        if p_min is None:
            p_min = self.ranges['screen_y_min']
        if p_max is None:
            p_max = self.ranges['screen_y_max']
            
        pixel_values = []
        for m in meter_values:
            p = p_min + ((m - m_min) / (m_max - m_min)) * (p_max - p_min)
            if m > 0.3:
                p = 600 - (p - 600)
            else:
                p = 600 + (600 - p)
            pixel_values.append(p)
        return pixel_values
    
    def robot_pose_to_screen_coords(self, robot_pose):
        """
        Convert robot pose to screen coordinates using the scaling method
        from the original position callback
        
        Args:
            robot_pose: Robot pose message with position attributes
            
        Returns:
            Tuple of (screen_x, screen_y) coordinates
        """
        screen_x = (robot_pose.position.y * self.scale_x) + self.offset_x
        screen_y = (-robot_pose.position.z * self.scale_y) + self.offset_y
        
        # Clamp to screen bounds
        screen_x = max(0, min(1920 - 30, screen_x))  # 30 is icon radius
        screen_y = max(0, min(1080 - 30, screen_y))  # 1080 not 1420
        
        return int(screen_x), int(screen_y)
    
    def get_target_pixel_position(self, target_pose):
        """
        Convert target pose to pixel coordinates using the spline conversion method
        
        Args:
            target_pose: Target pose to convert
            
        Returns:
            Tuple of (target_x_pixel, target_y_pixel)
        """
        target_x_pixel = self.convert_to_pixels_x([target_pose.position.y])[0]
        target_y_pixel = self.convert_to_pixels_y([target_pose.position.z])[0]
        return int(target_x_pixel), int(target_y_pixel)
    
    def get_display_waypoints(self, waypoints):
        """
        Convert a list of waypoint poses to display coordinates
        
        Args:
            waypoints: List of pose waypoints
            
        Returns:
            List of (x, y) pixel coordinate tuples for display
        """
        y_coords = [wp.position.y for wp in waypoints]
        z_coords = [wp.position.z for wp in waypoints]
        
        x_pixels = self.convert_to_pixels_x(y_coords)
        y_pixels = self.convert_to_pixels_y(z_coords)
        
        return list(zip(x_pixels, y_pixels))