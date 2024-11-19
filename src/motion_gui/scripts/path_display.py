#!/usr/bin/env python
import pygame
from random import randint
import numpy as np
from scipy.interpolate import splprep, splev
import rospy
from geometry_msgs.msg import Pose2D

#-------------------
#Initializing Window
#-------------------
pygame.init()
window = pygame.display.set_mode((1920,1420)) #setting window size
background_color = (2, 114, 212)
window.fill(background_color)
pygame.display.set_caption("Sawyer's Path")

# TUPLE LIST FOR STORING X AND Y COORDINATES TO SEND TO SAWYER
path_coordinates = []

#-------------------
#Creating points limits for gui
#-------------------
x_screen_limit_left = 200
x_screen_limit_right = 1720
y_screen_limit_top = 200
y_screen_limit_bottom = 1000

#X-coords of points: Starting at 200, in intervals of 380 pixels
x_coordinate_interval = 190
x_coordinate_interval = 380

def create_spline():
    #list of 5 points (tuples) used to create curve path
    points = [(200 + i * x_coordinate_interval, randint(y_screen_limit_top, y_screen_limit_bottom) if 0 < i < 4 else 600) for i in range(5)] #setting first and last point to middle height

    #drawing coordinates and points
    for index, point in enumerate(points):

        #drawing coordinate points
        pygame.draw.circle(window, (255, 255, 255), [points[index][0], points[index][1]], 10, 0) #0 line width makes a filled in circle

        #connecting dots with linear lines
        pygame.draw.lines(window, (255, 255, 255), False, points, 2) # False means the line is not closed, 2 is the width

        #-------------------
        #Spline interpolation to smooth the line
        #-------------------
        x_coords, y_coords = zip(*points) #separates x and y values of points into separate objects
        tck, u = splprep([x_coords, y_coords], s=2) #tck is 'tuple of spline representation', nessecary data to define the curve. 'u' is just parameters used in interpolation
        u_fine = np.linspace(0, 1, 1000) #'u_fine' is parameters for the new points of the smooth curves. 'np.linespace' generates 1000 evenly spaced values between 0 and 1
        x_fine, y_fine = splev(u_fine, tck) #new set of curve points

        # Draw the smooth line
        smooth_points = list(zip(x_fine, y_fine)) #packaging new curve points back into tuples
        
        global path_coordinates
        path_coordinates = [(x/2000 - 0.1, -y/1500 + 0.4) for x, y in smooth_points] #list of coords in meters, 1:2 scale. Zeroed by offset 0.1 for X and 0.3 for Y and sent to sawyer

        #setting and printing real coordinate values in cm
        pygame.draw.lines(window, (255, 0, 0), False, smooth_points, 4)  # False means the line is not closed, 4 is the width


#-------------------
# PUBLISHER ~~topic: coordinates~~ FOR SAWYER_CARTESIAN_MOTION.PY to use 
#-------------------
def coordinatePublisher():
    rospy.init_node('coordinatePublisher')
    coordinate_publisher = rospy.Publisher('coordinates', Pose2D, queue_size=10)
    rate = rospy.Rate(1000) #1000 Hz

    for coord in path_coordinates:
        pose = Pose2D()
        pose.x = coord[0]
        pose.y = coord[1]
        pose.theta = 0.0

        rospy.loginfo(f"Publishing: ROBOT_X: {pose.x}, ROBOT_Y: {pose.y}")
        coordinate_publisher.publish(pose)
        rate.sleep()

def main():

    try:
        create_spline()
        pygame.display.update()
        coordinatePublisher()
        pygame.display.update()
    except rospy.ROSInterruptException:
        print("Error running the publisher")
        pass
   