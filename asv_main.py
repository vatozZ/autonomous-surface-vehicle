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
from path_planner import * 
from plot_utils import *

"""
class LidarSensor:
    def __init__(self):
        self.one_scan = []
        self.step = 0
        self.PORT_NAME = '/dev/ttyUSB0'
        self.lidar = RPLidar(None, self.PORT_NAME, timeout=3)
        self.zk = []
        
    def get_measurement(self):
        for scan in self.lidar.iter_scans():
            self.zk = scan
            return self.zk"""            

def LidarSensor():
    global zk
    port_name = '/dev/ttyUSB0'
    lidar = RPLidar(None, port_name, timeout=3)
    time.sleep(0.2)
    while True:
        for scan in lidar.iter_scans():
            zk = scan

def process_scan(zk):

    zk_temp = np.ones(shape=(1, 360)) * lidar_max_range
    
    for i in zk:
        _, theta_, range_ = i[0], i[1], i[2]
        theta_ = int(theta_)
        zk_temp[0][theta_] = range_ / 1000.0 #mm to m
    
    return zk_temp

def main():

    global path_calculate_flag, program_timer, take_off_timer_t1, take_off_timer_t0
    global path_success, lidar, zk, motor_started, motor_started_trigger, flag_cnt
    global motor_t0_starter, motor_t1_starter
    
    t0 = time.time()
    t1 = time.time()

    Xkk = reset_init(n_particles=n_particles, region_mode=region_mode)
    wkk = np.ones(n_particles) / n_particles

    program_timer = time.time()

    input("Program is ready. Enter for run...")

    while True:
        
        #time.sleep(0.001)
        
        #print("--")
        
        t1 = time.time()
        dt = t1 - t0 
        t0 = t1
        
        #print("dt", round(dt,1))

        """try:
            zk = lidar.zk
            
        except:
            lidar.stop()
            lidar.disconnect()
            lidar = LidarSensor()
            thr = threading.Thread(target=lidar.get_measurement)
            thr.start()
            
            continue"""
        
        zk = process_scan(zk=zk)
        
        zk = zk.reshape(360, -1)
        
        #print("zk[:10]", zk[:10])
        
        if np.all(zk == lidar_max_range):
            print("no any meaningful measurement has taken. take measurement again.")
            continue
        
        Xkk, wkk, likelihood_array = update(Xkk, zk, wkk)
        
        Xkk = predict(Xkk, dt)
        
        #get estimate.
        est_X = predict(Xkk, dt)
        
        Xkk_MAP, Xkk_MMSE = estimate_target_state(Xkk=est_X, wkk=wkk,lk=likelihood_array)

        if estimate_type.lower() == 'mmse':
            Xkk_est = Xkk_MMSE
        
        elif estimate_type.lower() == 'map':
            Xkk_est = Xkk_MAP
        
        #print("Estimate:", Xkk_est)
                
        vehicle_x, vehicle_y, vehicle_theta = Xkk_est[0], Xkk_est[1], Xkk_est[4]
        
        #vehicle_x, y metre, theta = (-179, 179) radar koordinat sisteminde 

        if path_calculate_flag == True:
            path = get_path(start=(vehicle_x, vehicle_y), goal=None, obstacles=None)
            target_position = np.array([path.coords.xy[0][-1], path.coords.xy[1][-1]])
            path_calculate_flag = False
        
        vehicle_position = np.array([Xkk_est[0], Xkk_est[1]])
        
        visualize_flag = True
        
        if visualize_flag == True:
            
            plt.clf()
            plt.title('Self-Localization')

            blue_patch = patches.Patch(color='blue', label='Estimate')
            green_patch = patches.Patch(color='green', label='Ground-Truth')
    
            plt.legend(handles=[green_patch, blue_patch], loc='upper right')
            
            axis = plt.gca()
            
            axis = visualize(Xkk=Xkk, X_MMSE=Xkk_MMSE, X_MAP=Xkk_MAP, show_particles=show_particles, axis=axis)
        
            plt.plot(*path.coords.xy, color='g', label='path')
            plt.legend()
            plt.pause(1e-6)
            
        take_off_timer_t1 = time.time()
        
        if take_off_timer_t1 - take_off_timer_t0 > take_off_duration_for_filter: 
            
            if flag_cnt == True:

                motor_started = time.time()
                flag_cnt = False
                
            if time.time() - motor_started < 15.:
                    path_success = path_follow(vehicle_position, Xkk_est[4], path, target_position)
                    path_success = True
                    motor_t0_starter = time.time()
                    #time.sleep(0.3)
             
            else:
                turn_wheels(direction='stop')
                
        else:
            continue
        
        if not path_success:
            print("Reset Filter.")
            Xkk = reset(n_particles=n_particles)
            wkk = np.ones(n_particles) / n_particles
        

if __name__ == "__main__":
    
    
    #lidar = LidarSensor()
    thr = threading.Thread(target=LidarSensor)
    thr.start()

    scan_data = [0]*360

    send_string = 's'

    take_off_duration_for_filter = 19.0

    path_calculate_flag = True

    robot_pose_x, robot_pose_y, yaw, gt = 0., 0., 0., 0.
    
    zk = np.empty(shape=(1, 360))

    take_off_timer_t0 = time.time()
    take_off_timer_t1 = time.time()

    motor_timer_0 = time.time()
    motor_timer_1 = time.time()
    
    path_success = False
    
    motor_t0_starter = time.time()
    motor_t1_starter = time.time()

    motor_started = time.time()
    flag_cnt = True
    
    main()
