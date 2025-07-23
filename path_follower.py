from shapely.geometry import LineString, Point
import time 
import numpy as np
import shapely.geometry as sg
import matplotlib.pyplot as plt
from constants import *
from particle_filter import *
from surface_vt import *

def select_nearest_target(circle, line):
    intersection_points = line.intersection(circle)

    x, y = intersection_points.coords.xy
    x = list(x)

    if len(x) == 1:
        closest_point = intersection_points
    else:
        closest_point = intersection_points.coords[-1]
        
    return closest_point


def control_intersection(vehicle, road):
    
    if type(road) == list:
        road = LineString(road)
    
    if type(vehicle) == list:
        vehicle = Point(vehicle[0], vehicle[1])
        vehicle.buffer(look_ahaed_point)

    if vehicle.intersects(road) == False:
        while not vehicle.intersects(road):
            vehicle = vehicle.buffer(look_ahaed_point)
            if vehicle.intersects(road):
                return select_nearest_target(vehicle, road)
    else:
        return select_nearest_target(vehicle, road)
        
def stop_engine():
    turn_wheels(direction='stop')

def turn_wheels(direction='forward'):
    global ser

    if direction == 'left':
        send_string = ("a")
        ser.write(send_string.encode('utf-8'))

    elif direction == 'right':
        send_string = ("d")
        ser.write(send_string.encode('utf-8'))

    elif direction == 'forward':
        send_string = ("w")
        ser.write(send_string.encode('utf-8'))
        
    elif direction == 'stop':
        send_string = ("s")
        ser.write(send_string.encode('utf-8'))
        
    print("Command: ", send_string)
    
    
def engine(vehicle_heading, target_radyan):

    _, vehicle_heading = adjust_angles(0.0, np.pi/2 - vehicle_heading)

    target_degree = np.mod(np.rad2deg(target_radyan), 360)

    heading_degree = np.mod(np.rad2deg(vehicle_heading), 360)

    rotation = np.mod(target_degree - heading_degree, 360)

    
    if rotation > 0.0 and rotation < 180.0:
        turn_wheels(direction='left')
    
    elif rotation > 180 and rotation < 360.0:
        turn_wheels(direction='right')
    else:
        turn_wheels(direction='forward')
        
def path_follow(vehicle_position, vehicle_heading, path, target_position):

    global program_timer

    circle = control_intersection(Point(vehicle_position[0], vehicle_position[1]), path)

    if np.ndim(circle) != 1:
        return
    
    target_vector = np.array([circle[0]-vehicle_position[0], circle[1]-vehicle_position[1]])

    target_unit_vector = target_vector / np.linalg.norm(target_vector + 1e-10)
    
    target_radyan = np.arctan2(target_unit_vector[1], target_unit_vector[0])
    
    distance = np.linalg.norm(vehicle_position- target_position)
    
    
    if distance <= distance_target_tolerance: #cm
        if time.time() - program_timer >= 5.0:
            print("REACHED.")
            stop_engine()
            quit()
            return True
        
        else:
            return False
    
    else:
        if vt == 0:
            
            engine(vehicle_heading, target_radyan)
            """go_with(direction='forward', duration=2)
            go_with(direction='left', duration=2)
            go_with(direction='right', duration=1)
            go_with(direction='stop', duration=10)"""
            
        elif vt == 1:
            engine_vt(vehicle_heading, target_radyan, mode_)
            
        return True
        
