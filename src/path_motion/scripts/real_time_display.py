#!/usr/bin/env python

import pygame
import rospy
import threading
from intera_core_msgs.msg import EndpointState
from random import randint
import numpy as np
from scipy.interpolate import splprep, splev



# TUPLE LIST FOR STORING X AND Y COORDINATES TO SEND TO SAWYER
path_coordinates = []

#Creating points limits for gui
x_screen_limit_left = 200
x_screen_limit_right = 1720
y_screen_limit_top = 200
y_screen_limit_bottom = 1000

#X-coords of points: Starting at 200, in intervals of 380 pixels
x_coordinate_interval = 190
x_coordinate_interval = 380



'''
Curve and point generation
'''
class CoordinateConverter:
    def __init__(self):
        # Screen dimensions in pixels
        self.x_screen_limit_left = x_screen_limit_left
        self.x_screen_limit_right = x_screen_limit_right
        self.y_screen_limit_top = y_screen_limit_top
        self.y_screen_limit_bottom = y_screen_limit_bottom
        
        # Desired robot workspace dimensions in centimeters
        self.x_robot_range = 75  # 75cm total horizontal range
        self.y_robot_range = 50  # 50cm total vertical range
        
        # Calculate conversion factors
        self.x_pixel_range = self.x_screen_limit_right - self.x_screen_limit_left
        self.y_pixel_range = self.y_screen_limit_bottom - self.y_screen_limit_top
        
        self.x_scale = self.x_robot_range / self.x_pixel_range
        self.y_scale = self.y_robot_range / self.y_pixel_range

    def pixels_to_cm(self, points):
        """Convert pixel coordinates to centimeter coordinates."""
        cm_points = []
        
        for px, py in points:
            # Normalize coordinates to 0-1 range
            x_normalized = (px - self.x_screen_limit_left) / self.x_pixel_range
            y_normalized = (py - self.y_screen_limit_top) / self.y_pixel_range
            
            # Convert to centimeters, ranging from 0 to max
            x_cm = (x_normalized * self.x_robot_range) / 100  # Will range from 0 to 75cm (left to right)
            y_cm = - (y_normalized * self.y_robot_range - 25) / 100 # Will range from 0 to +=25cm
            
            cm_points.append((x_cm, y_cm))
            
        return cm_points
    


    


class SawyerVisualizer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('positionSubscriber', anonymous=True)
        
        # Initialize Pygame
        pygame.init()
        self.window = pygame.display.set_mode((1920, 1420))
        self.background_color = (2, 114, 212)
        pygame.display.set_caption("Sawyer Real-Time Display")

        converter = CoordinateConverter()
        
        #list of 5 points (tuples) used to create curve path
        self.points = [(200 + i * x_coordinate_interval, randint(y_screen_limit_top, y_screen_limit_bottom) if 0 < i < 4 else 600) for i in range(5)] #setting first and last point to middle height
        
        #Spline interpolation to smooth the line
        # x_coords, y_coords = zip(self.points) #separates x and y values of points into separate objects
        x_coords = [p[0] for p in self.points]  # Get x coordinates
        y_coords = [p[1] for p in self.points]  # Get y coordinates
        
        tck, u = splprep([x_coords, y_coords], s=2) #tck is 'tuple of spline representation', nessecary data to define the curve. 'u' is just parameters used in interpolation
        u_fine = np.linspace(0, 1, 1000) #'u_fine' is parameters for the new points of the smooth curves. 'np.linespace' generates 1000 evenly spaced values between 0 and 1
        x_fine, y_fine = splev(u_fine, tck) #new set of curve points

        #draw the smooth line
        self.smooth_points = list(zip(x_fine, y_fine)) #packaging new curve points back into tuples
        
        global path_coordinates
        path_coordinates = converter.pixels_to_cm(self.smooth_points)

        #setting and printing real coordinate values in cm
        # pygame.draw.lines(self.window, (255, 0, 0), False, self.smooth_points, 4)  # False means the line is not closed, 4 is the width


        # Plot with Matplotlib to show centimeter values
        # plt.figure(figsize=(8, 6))
        # x_cm, y_cm = zip(*path_coordinates)
        # plt.plot(x_cm, y_cm, color='green', label='Robot Path')
        # plt.title('Robot Path in Centimeters')
        # plt.xlabel('X (cm)')
        # plt.ylabel('Y (cm)')
        # plt.grid(True)
        # plt.legend()
        
            # Set axis limits to show full range
        # plt.xlim(0, 75)
        # plt.ylim(0, 50)
        # plt.axis('equal')  # Make the plot scale equally
        # plt.show()

        
        # Rectangle properties
        self.rect_color = (9, 179, 54)
        self.rect_width = 50
        self.rect_height = 50
        
        # Set initial position to (200, 600)
        self.x = 200
        self.y = 600
        self.position_initialized = True
        
        # Scale factors to convert robot coordinates to screen coordinates
        self.scale_x = 1000  # pixels per meter
        self.scale_y = 1000  # pixels per meter
        
        # Threading lock for position updates
        self.lock = threading.Lock()
        
        # Start ROS subscriber
        self.subscriber = rospy.Subscriber("/robot/limb/right/endpoint_state",
                                         EndpointState,
                                         self.position_callback)
    
    def draw_path(self):
        # Draw the coordinate points
        for point in self.points:
            pygame.draw.circle(self.window, (255, 255, 255), point, 10, 0)

        # Draw lines connecting the points
        pygame.draw.lines(self.window, (255, 255, 255), False, self.points, 2)

        # Draw the smooth spline
        pygame.draw.lines(self.window, (255, 0, 0), False, self.smooth_points, 4)


    def position_callback(self, msg):
        with self.lock:
            # Convert robot coordinates to screen coordinates
            screen_x = (msg.pose.position.x * self.scale_x)
            screen_y = (-msg.pose.position.y * self.scale_y)
            
            # Clamp values to screen boundaries
            self.x = max(0, min(1920 - self.rect_width, screen_x))
            self.y = max(0, min(1080 - self.rect_height, screen_y))
    
    def run(self):
        clock = pygame.time.Clock()
        running = True
        
        while running and not rospy.is_shutdown():
            # Handle PyGame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            
            # Clear screen with background color
            self.window.fill(self.background_color)

            # Draw the path first (so it appears behind the rectangle)
            self.draw_path()
            
            # Draw rectangle at current position
            with self.lock:
                pygame.draw.rect(self.window,
                               self.rect_color,
                               (int(self.x), int(self.y),
                                self.rect_width, self.rect_height))
            
            # Update display
            pygame.display.flip()
            
            # Control frame rate
            clock.tick(60)
        
        pygame.quit()
        rospy.signal_shutdown("PyGame window closed")

if __name__ == "__main__":
    try:
        visualizer = SawyerVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pygame.quit()