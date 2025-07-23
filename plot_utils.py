#!/usr/bin/env python3
# -*- coding:latin-1 -*-

import time 
import shapely.geometry as sg
import matplotlib.pyplot as plt
from particle_filter import * 
import numpy as np
from constants import *
from shapely.geometry import LineString, Point
import serial
from adafruit_rplidar import RPLidar
import threading
from path_follower import *
from constants import *


def visualize(Xkk, X_MMSE, X_MAP, show_particles, axis):
    
    coordinates = ((0.1, 0.1), (0.1, 4.9), (4.9, 4.9), (4.9, 0.1))
    wall = sg.Polygon(coordinates) #example.
    x, y = wall.exterior.xy
    
    axis.plot(x, y, c="black")
    
    particle_x_coords, particle_y_corods = Xkk[0,:], Xkk[1, :]
    
    if show_particles:
        axis.scatter(particle_x_coords, particle_y_corods, c='darkviolet')
    
    width, length = 0.05, 0.25
    
    # MMSE Estimation
    plt.arrow(X_MMSE[0], X_MMSE[1], 
        dx=np.sin(X_MMSE[4]) * length, 
        dy=np.cos(X_MMSE[4]) * length, 
        width=width, facecolor='b',edgecolor='b', label='Estimate')

    # MAP Estimation  
    #plt.arrow(X_MAP[0], X_MAP[1], np.sin(X_MAP[4])* length, np.cos(X_MAP[4])* length, width=width, facecolor='r', edgecolor='r', label='MAP')
    
    #plt.legend(handles=[royal_patch, blue_patch, green_patch, violet_patch, red_patch], loc='upper right')
    
    return axis
    
