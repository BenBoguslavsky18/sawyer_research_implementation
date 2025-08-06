#!/usr/bin/env python

"""
Configuration file for Sawyer Robot Challenge
Contains all constants, parameters, and level definitions
"""

from geometry_msgs.msg import Pose, Point, Quaternion

# Screen display limits
X_SCREEN_LIMIT_LEFT = 200
X_SCREEN_LIMIT_RIGHT = 1720
Y_SCREEN_LIMIT_TOP = 200
Y_SCREEN_LIMIT_BOTTOM = 1000

# Game timing constants
GAME_DURATION = 30.0  # 30 seconds
NUM_LEVELS = 6  # Number of spline challenges

# Robot control parameters
LOW_STIFFNESS = [50.0, 50.0, 50.0, 500.0, 500.0, 500.0]  # Increased X stiffness to prevent drift

# Motion parameters
NORMAL_MODE_SPEED = {
    'max_linear_speed': 0.001,      # 1mm/s - extremely slow
    'max_linear_accel': 0.001,      # Very gentle acceleration
    'max_rotational_speed': 0.001,
    'max_rotational_accel': 0.001,
    'max_joint_speed_ratio': 0.05   # Very slow joint motion
}

TRACKING_MODE_SPEED = {
    'max_linear_speed': 0.02,       # 20mm/s - slow but faster than normal mode
    'max_linear_accel': 0.02,       # Gentle acceleration
    'max_rotational_speed': 0.02,
    'max_rotational_accel': 0.02,
    'max_joint_speed_ratio': 0.1    # Moderate joint motion speed
}

HOLD_POSITION_SPEED = {
    'max_linear_speed': 0.001,
    'max_linear_accel': 0.001,
    'max_rotational_speed': 0.001,
    'max_rotational_accel': 0.001,
    'max_joint_speed_ratio': 0.01
}

MOVE_TO_START_SPEED = {
    'max_linear_speed': 0.1,
    'max_linear_accel': 0.1,
    'max_rotational_speed': 0.1,
    'max_rotational_accel': 0.1,
    'max_joint_speed_ratio': 0.1
}

# Coordinate conversion parameters
COORDINATE_RANGES = {
    'robot_y_min': -0.3,
    'robot_y_max': 0.5,
    'robot_z_min': 0.1,
    'robot_z_max': 0.5,
    'screen_x_min': 200,
    'screen_x_max': 1720,
    'screen_y_min': 200,
    'screen_y_max': 1000
}

# Game parameters
SUCCESS_RADIUS = 50  # pixels - radius for success detection
AUTO_CLOSE_DELAY = 5.0  # Show results for 5 seconds before closing

# UI parameters
ROBOT_ICON_RADIUS = 30
SCALE_X = 1900
SCALE_Y = 1950
ROBOT_POSITION_OFFSET_X = 770
ROBOT_POSITION_OFFSET_Y = 1180

# Colors
BACKGROUND_COLOR = (2, 114, 212)
ROBOT_COLOR_DEFAULT = (9, 179, 54)
ROBOT_COLOR_WON = (0, 255, 0)
ROBOT_COLOR_NORMAL_MODE = (255, 165, 0)  # Orange
ROBOT_COLOR_TRACKING_MODE = (255, 0, 255)  # Magenta
SPLINE_PATH_COLOR = (255, 0, 0)
TRAIL_COLOR = (0, 0, 0)
TARGET_SUCCESS_COLOR = (0, 255, 0)
TARGET_CENTER_COLOR = (255, 255, 0)

# Define 6 different spline challenges with varying difficulty
SPLINE_CHALLENGES = [
    # Level 1: Simple curved spline
    {
        'start': Pose(position=Point(x=0.75, y=-0.3, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'end': Pose(position=Point(x=0.75, y=0.5, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'control_points': [[0.75, -0.3, 0.3], [0.75, -0.1, 0.4], [0.75, 0.1, 0.2], [0.75, 0.3, 0.4], [0.75, 0.5, 0.3]],
        'name': "Level 1: Curved Spline",
        'type': 'spline'
    },
    # Level 2: Different curved spline
    {
        'start': Pose(position=Point(x=0.75, y=-0.3, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'end': Pose(position=Point(x=0.75, y=0.5, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'control_points': [[0.75, -0.3, 0.3], [0.75, -0.1, 0.2], [0.75, 0.1, 0.4], [0.75, 0.3, 0.2], [0.75, 0.5, 0.3]],
        'name': "Level 2: Alternate Spline",
        'type': 'spline'
    },
    # Level 3: Straight line segments
    {
        'start': Pose(position=Point(x=0.75, y=-0.3, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'end': Pose(position=Point(x=0.75, y=0.5, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        'control_points': [[0.75, -0.3, 0.3], [0.75, -0.1, 0.5], [0.75, 0.1, 0.15], [0.75, 0.3, 0.35], [0.75, 0.5, 0.3]],
        'name': "Level 3: Linear Segments",
        'type': 'linear'
    },
    # Level 4: Complex maze
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
    # Level 5: Simple maze
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
    # Level 6: Zigzag with coverage tracking
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