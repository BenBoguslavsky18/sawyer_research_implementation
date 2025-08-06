#!/usr/bin/env python

"""
Spline and path generation for Sawyer Robot Challenge
Handles creation of smooth trajectories and linear paths
"""

import numpy as np
import rospy
from scipy.interpolate import CubicSpline
from geometry_msgs.msg import Pose, Point


class SplineGenerator:
    """Generates smooth splines and linear paths for robot trajectories"""
    
    def __init__(self):
        pass
    
    def generate_spline_waypoints_custom(self, start_pose, end_pose, control_points_list, spline_type='spline', num_points=40):
        """
        Generate spline waypoints using custom control points and type
        
        Args:
            start_pose: Starting pose for the trajectory
            end_pose: Ending pose for the trajectory
            control_points_list: List of control points defining the path
            spline_type: 'spline' for smooth curves, 'linear' for straight segments
            num_points: Number of waypoints to generate for spline type
            
        Returns:
            Tuple of (control_points_array, waypoint_poses, display_waypoints)
        """
        # Build control points array exactly like the reference code
        control_points = np.array([
            [start_pose.position.x, start_pose.position.y, start_pose.position.z],
            *control_points_list[1:-1],  # Use middle control points from the list
            [end_pose.position.x, end_pose.position.y, end_pose.position.z]
        ])
        
        rospy.loginfo(f"Control points array: {control_points}")

        if spline_type == 'spline':
            waypoints, display_waypoints = self._create_cubic_spline(
                control_points, start_pose.orientation, num_points
            )
        else:  # 'linear' type
            waypoints, display_waypoints = self._create_linear_path(
                control_points, start_pose.orientation
            )

        rospy.loginfo(f"Generated {len(waypoints)} waypoints")
        rospy.loginfo(f"First waypoint: x={waypoints[0].position.x:.3f}, y={waypoints[0].position.y:.3f}, z={waypoints[0].position.z:.3f}")
        rospy.loginfo(f"Last waypoint: x={waypoints[-1].position.x:.3f}, y={waypoints[-1].position.y:.3f}, z={waypoints[-1].position.z:.3f}")
        
        return control_points, waypoints, display_waypoints
    
    def _create_cubic_spline(self, control_points, orientation, num_points):
        """
        Create smooth cubic spline trajectory
        
        Args:
            control_points: Numpy array of control points
            orientation: Quaternion orientation to use for all waypoints
            num_points: Number of waypoints to generate
            
        Returns:
            Tuple of (waypoint_poses, display_waypoints)
        """
        # Use cubic spline interpolation (smooth curves)
        t = np.linspace(0, 1, len(control_points))
        t_spline = np.linspace(0, 1, num_points)
        
        x_spline = CubicSpline(t, control_points[:, 0])(t_spline)
        y_spline = CubicSpline(t, control_points[:, 1])(t_spline)
        z_spline = CubicSpline(t, control_points[:, 2])(t_spline)

        # Generate higher resolution for display
        t_spline_display = np.linspace(0, 1, 1000)
        y_spline_display = CubicSpline(t, control_points[:, 1])(t_spline_display)
        z_spline_display = CubicSpline(t, control_points[:, 2])(t_spline_display)

        # Generate waypoint poses
        waypoints = [
            Pose(position=Point(x=x, y=y, z=z), orientation=orientation)
            for x, y, z in zip(x_spline, y_spline, z_spline)
        ]
        
        display_waypoints = list(zip(y_spline_display, z_spline_display))
        
        return waypoints, display_waypoints
    
    def _create_linear_path(self, control_points, orientation):
        """
        Create linear path with straight line segments between points
        
        Args:
            control_points: Numpy array of control points
            orientation: Quaternion orientation to use for all waypoints
            
        Returns:
            Tuple of (waypoint_poses, display_waypoints)
        """
        # Use direct linear interpolation between points (straight line segments)
        x_path = control_points[:, 0]
        y_path = control_points[:, 1] 
        z_path = control_points[:, 2]

        waypoints = [
            Pose(position=Point(x=x, y=y, z=z), orientation=orientation)
            for x, y, z in zip(x_path, y_path, z_path)
        ]

        # Interpolation for display - create smooth lines between waypoints
        y_path_display = []
        z_path_display = []
        
        for i in range(len(y_path) - 1):
            segment_points = 100  # Points per segment for display
            y_interpolated = np.linspace(y_path[i], y_path[i+1], segment_points, False)
            z_interpolated = np.linspace(z_path[i], z_path[i+1], segment_points, False)
            
            y_path_display.extend(y_interpolated)
            z_path_display.extend(z_interpolated)

        display_waypoints = list(zip(y_path_display, z_path_display))
        
        return waypoints, display_waypoints
    
    def create_hold_position_waypoint(self, current_pose, orientation):
        """
        Create a single waypoint at the current position for holding
        
        Args:
            current_pose: Current robot pose
            orientation: Desired orientation
            
        Returns:
            Single pose waypoint
        """
        return Pose(
            position=Point(
                x=current_pose.position.x,
                y=current_pose.position.y,
                z=current_pose.position.z
            ),
            orientation=orientation
        )