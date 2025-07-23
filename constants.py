#!/usr/bin/env python3

import shapely.geometry as sg    
import numpy as np
import serial

n_particles = 150 # number of particles

Qk = [0.05, 0.05, 0.05, 0.01, 2*np.pi/180, 0.5*np.pi/180] # [px,py,v,a,theta,w] Qk stands for process noise

#Qk = [0.2, 0.2, 0.1, 0.5, 2*np.pi/180, 0.5*np.pi/180] # [px,py,v,a,theta,w] #this initial parameters also work.

range_error = 0.1 #meter

visualize_flag = True

show_particles = 1 # True or False 

look_ahaed_point = 1.0 # How much distance needs to be tracked by the pure pursuit path follower.

distance_target_tolerance = 0.8 # if 80 centimeter is left until the target location, stop the vehicle, since this is a buffer zone for the vehicle body.

estimate_type = 'mmse' # or 'map'

lidar_max_range = 12.0 #from the datasheet

ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=1) # serial communication settings.
ser.flush() # flush the port.

# obstacles need to be assigned as Polygons.
wall_1 = sg.LineString([(0.11, 0.11), (0.11, 4.89)])
wall_2 = sg.LineString([(0.11, 4.89), (4.89, 4.89)])
wall_3 = sg.LineString([(4.89, 4.89), (4.89, 0.11)])
wall_4 = sg.LineString([(4.89, 0.11), (0.11, 0.11)])
box = sg.Polygon([(0.2, 4.465), (0.6, 4.465), (0.6, 4.735), (0.2, 4.735), (0.2, 4.465)])
obstacles = [wall_1, wall_2, wall_3, wall_4, box]

vt = 0 #0: autonomous, 1: manual
mode_ = 3

# If particle doesn't converge initially, initial particles can be assigned to a specific area, instead of the whole region.
region_mode = 1 #1- left-bottom, 2- right-bottom of the pool
