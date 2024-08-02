#!/usr/bin/env python
import pygame
from random import randint
import numpy as np
from scipy.interpolate import splprep, splev
import rospy


#---Initializing Window---
pygame.init()
window = pygame.display.set_mode((1920,1420)) #setting window size
background_color = (2, 114, 212)
window.fill(background_color)
pygame.display.set_caption("Ben\'s Window")


#---Drawing black rectangle and general font settings---
pygame.draw.rect(window, (0,0,0), (0, 0, 1920, 100)) #adding black rectangle
font = pygame.font.Font(None, 30) #font settings


#---Creating points and line connection---
x_screen_limit_left = 200
x_screen_limit_right = 1720
y_screen_limit_top = 200
y_screen_limit_bottom = 1000

#X-coords of points: Starting at 210, in intervals of 187.5 pixels
x_coordinate_interval = 190

#list of 8 points (tuples) used to create curve path
points = [(200 + i * x_coordinate_interval, randint(y_screen_limit_top, y_screen_limit_bottom)) for i in range(9)] 

#drawing coordinates and points
for index, point in enumerate(points):
    #listing coordinate values in black rectangle
    point_text = font.render("(" + str(points[index][0]) + ", " + str(points[index][1]) + ")", True, (255, 255, 255)) #text with smoothing
    point_rect = point_text.get_rect(center=(400 + index*125, 50)) 
    window.blit(point_text, point_rect) #blit is 'block transfer' -- puts point_text to point_rect

    #drawing coordinate points
    pygame.draw.circle(window, (255, 255, 255), [points[index][0], points[index][1]], 10, 0) #0 line width makes a filled in circle

#connecting dots with linear lines
pygame.draw.lines(window, (255, 255, 255), False, points, 2) # False means the line is not closed, 2 is the width



#---Spline interpolation to smooth the line---
x_coords, y_coords = zip(*points) #separates x and y values of points into separate objects
tck, u = splprep([x_coords, y_coords], s=2) #tck is 'tuple of spline representation', nessecary data to define the curve. 'u' is just parameters used in interpolation
u_fine = np.linspace(0, 1, 1000) #'u_fine' is parameters for the new points of the smooth curves. 'np.linespace' generates 1000 evenly spaced values between 0 and 1
x_fine, y_fine = splev(u_fine, tck) #new set of curve points

# Draw the smooth line
smooth_points = list(zip(x_fine, y_fine)) #packaging new curve points back into tuples
pygame.draw.lines(window, (255, 0, 0), False, smooth_points, 4)  # False means the line is not closed, 2 is the width



#---main loop---
while True:

    #preprocessing of inputs
    eventList = pygame.event.get()
    for event in eventList:
        if event.type == pygame.QUIT:
            pygame.quit()


    #rendering
    pygame.display.update()      