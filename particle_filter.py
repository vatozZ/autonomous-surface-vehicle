# -*- coding:latin-1 -*-
#!/usr/bin/env python3

"""
14.August.2023
Particle-Filter for self-localisation
Provided by @VatozZ
"""

import numpy as np 
from numpy.random import uniform
import shapely.geometry as sg
import matplotlib.pyplot as plt
from matplotlib import patches
from  scipy.stats import multivariate_normal
from constants import *

n_measurement_dims = 5 #number of measurements to be taken

"""
Tune parameters:
1) n_particles 
2) range_error covariance for calculate_likelihood()
"""

def adjust_angles(a_ref, a, unit='rad'):
    # align angles in a w.r.t. a_ref
    if unit.lower() == 'deg':
        a = np.deg2rad(a)
        a_ref = np.deg2rad(a_ref)

    a_ref = np.remainder(a_ref, 2 * np.pi)
    if a_ref > np.pi:
        a_ref = a_ref - 2 * np.pi

    a = np.remainder(a, 2 * np.pi)
    if type(a) == np.ndarray:
        idx = np.argwhere(a > np.pi)
        if n_array_elements(idx) > 0:
            a[idx] = a[idx] - 2 * np.pi
    else:
        if a > np.pi:
            a = a - 2 * np.pi
        # a[a > np.pi] = a[a > np.pi] - 2 * np.pi

    dif = a - a_ref
    if type(a) is np.ndarray:
        idx = np.argwhere(np.abs(dif) > np.pi)
        if n_array_elements(idx) > 0:
            signs = np.sign(dif)
            if type(a) == np.ndarray:
                a[idx] = a[idx] + (-signs[idx] * 2 * np.pi)
            else:
                a = a - signs * 2 * np.pi
    else:  # scalar
        if np.abs(dif) > np.pi:
            a = a - np.sign(dif) * 2 * np.pi

    if unit.lower() == 'deg':
        a = np.rad2deg(a)
        a_ref = np.rad2deg(a_ref)

    return a_ref, a

def n_array_elements(x):

    # return number of elements in an array
    # None and empty arrays evaluated
    return x.ndim and x.size

def reset(n_particles):
    print("reset")
    n_state_dims = 6
    xlim=[0.2, 4.8]
    ylim=[0.2, 1.0]
    vmax=0.05
    amax=0.01
    omega_max= 0.2 * np.pi/180  
    Xkk = np.array([uniform(low=xlim[0], high= xlim[1], size= n_particles), \
                    uniform(low=ylim[0], high= ylim[1], size= n_particles), \
                    uniform(low=0.0, high= vmax, size= n_particles), \
                    uniform(low= -amax, high= amax, size= n_particles), \
                    uniform(low=0.0, high=2 * np.pi, size= n_particles), \
                    uniform(low= -omega_max, high= omega_max, size= n_particles)]).reshape(n_state_dims,  n_particles)
    return Xkk 

def reset_init(n_particles, region_mode=1):
    print("reset")
    n_state_dims = 6
    
    if region_mode == 1:
        xlim=[0.2, 1.5]
        ylim=[0.2, 1.5]
        
    """if region_mode == 2:
        xlim=[3., 4.8]
        ylim=[0.2, 1.5]"""
    
    vmax=0.05
    amax=0.01
    omega_max= 0.2 * np.pi/180  
    Xkk = np.array([uniform(low=xlim[0], high= xlim[1], size= n_particles), \
                    uniform(low=ylim[0], high= ylim[1], size= n_particles), \
                    uniform(low=0.0, high= vmax, size= n_particles), \
                    uniform(low= -amax, high= amax, size= n_particles), \
                    uniform(low=0.0, high=2 * np.pi, size= n_particles), \
                    uniform(low= -omega_max, high= omega_max, size= n_particles)]).reshape(n_state_dims,  n_particles)
    return Xkk

def predict(xkk, dt=None):
    xkkm1 = ct_move(xkk, dt)
    xkkm1[4, :] = np.mod(xkkm1[4, :], 2 * np.pi)
    _, xkkm1[4, :] = adjust_angles(0.0, xkkm1[4, :].reshape(-1))
    xkkm1[5, :] = np.mod(xkkm1[5, :], 2 * np.pi)
    _, xkkm1[5, :] = adjust_angles(0.0, xkkm1[5, :].reshape(-1))
    return xkkm1

def ct_move(xkk, dt):
    x, y, v, a, phi, omega = xkk[0, :], xkk[1, :], xkk[2, :], xkk[3, :], xkk[4, :], xkk[5, :]
    xkkm1 = np.zeros_like(xkk)
    idx = np.where(np.abs(omega) <= np.finfo(float).eps)[0]  # zero
    idx_ = np.where(np.abs(omega) > np.finfo(float).eps)[0]  # nonzero
    xkkm1[0, idx_] = x[idx_] + (2 * v[idx_] * dt + a[idx_] * dt ** 2) / (omega[idx_] * dt) * np.sin(
        phi[idx_] + omega[idx_] * dt / 2) * np.sin(omega[idx_] * dt / 2)
    xkkm1[1, idx_] = y[idx_] + (2 * v[idx_] * dt + a[idx_] * dt ** 2) / (omega[idx_] * dt) * np.cos(
        phi[idx_] + omega[idx_] * dt / 2) * np.sin(omega[idx_] * dt / 2)
    xkkm1[0, idx] = x[idx] + (2 * v[idx] * dt + a[idx] * dt ** 2) * 0.5 * np.sin(phi[idx] + omega[idx] * dt / 2)
    xkkm1[1, idx] = y[idx] + (2 * v[idx] * dt + a[idx] * dt ** 2) * 0.5 * np.cos(phi[idx] + omega[idx] * dt / 2)
    xkkm1[2, :] = v + a * dt
    xkkm1[3, :] = a
    _, xkkm1[4, :] = adjust_angles(0.0, phi + omega * dt)
    xkkm1[5, :] = omega  # np.mod(omega, 2*np.pi)
    return xkkm1

def update(Xkk, zk, wkk):
    
    global n_measurement_dims
    zk = zk.astype('float')
    theta_array = np.where(zk !=  12.0)
    theta_array = theta_array[0]

    if theta_array.shape[0] > n_measurement_dims:
        theta_array = np.random.choice(theta_array, n_measurement_dims, replace=False)
    
    elif theta_array.shape[0] < n_measurement_dims and theta_array.shape[0] > 0:
        theta_array = np.random.choice(theta_array, n_measurement_dims, replace=True)

    if theta_array.shape[0] == 0:
        wkk = np.ones(len(Xkk[0,:])) * (1/(len(Xkk[0,:])))
    
    range_array = zk[theta_array] # selected angles: theta_array, and corresponding ranges
    #print("range_array," , range_array)
    zk_predicted = calculate_zk(Xkk, theta_array) # Predict the n_measurement_dims theta's ranges, for each particle
    
    likelihood_array = calculate_likelihood(zk=range_array, zk_predicted=zk_predicted)

    max_lk_particle = np.argmax(likelihood_array)
    
    #plt.scatter(Xkk[0, max_lk_particle], Xkk[1, max_lk_particle], c='r')
    
    #plt.pause(2)
    wkk = likelihood_array * wkk #Bayes: posterior = (likelihood * prior) / normalize(likelihood * prior)
    
    if np.sum(wkk.reshape(-1)) <= 0.001:
        Xkk = reset(n_particles=Xkk.shape[1])

    if np.sum(wkk.reshape(-1)) > 0.0:
        wkk = wkk / np.sum(wkk.reshape(-1))
        Xkk, wkk = resample(Xkk, wkk)

    elif np.sum(wkk.reshape(-1)) == 0.0:
        Xkk = reset(n_particles=Xkk.shape[1])

    else:
        Xkk = reset(n_particles=Xkk.shape[1])
        
    return Xkk, wkk, likelihood_array

def calculate_zk(Xkk, theta_array):

    n_particle = Xkk.shape[1] #columns: n.particles, rows: particles state

    theta_array = (theta_array/180) * np.pi
    lidar_max_range = 12.0

    measurement_matrix = np.empty(shape=(theta_array.shape[0], n_particle))

    for particle in range(n_particle):
        x, y, heading = Xkk[0, particle], Xkk[1, particle], Xkk[4, particle]
        """plt.clf()
        plt.scatter(Xkk[0, :], Xkk[1, :])
        """
        for bearing_number, bearing in enumerate(theta_array):
            dy = lidar_max_range * np.cos(heading + bearing)
            dx = lidar_max_range * np.sin(heading + bearing)
            laser = sg.LineString([(x,y), (x+dx, y+dy)])
            #plt.plot(laser.coords.xy[0], laser.coords.xy[1])

            range_list_for_all_obstacles = []
            
            for obstacle in obstacles:
                
                """try:
                    plt.plot(obstacle.exterior.xy[0], obstacle.exterior.xy[1], c='b')
                except:
                    plt.plot(*obstacle.xy, c='b')
                """
                intersect_point = obstacle.intersection(laser)
                
                if intersect_point.is_empty:
                    laser_range = lidar_max_range
                    #plt.text(x=laser.coords.xy[0][-1], y=laser.coords.xy[1][-1], s=str(12))
                else:
                    laser_range = intersect_point.distance(sg.Point(x,y))
                    #plt.text(x=intersect_point.coords.xy[0][0], y=intersect_point.coords.xy[1][0], s=str(round(laser_range,2)))
                range_list_for_all_obstacles.append(laser_range)
            
            """plt.xlim([-1, 6])
            plt.ylim([-1, 6])
            """
            range_for_given_bearing = min(range_list_for_all_obstacles) #minimum range from all controlled obstacles
            #plt.title
            #print("theta:", bearing*180/np.pi, " range:", range_for_given_bearing)
            measurement_matrix[bearing_number, particle] = range_for_given_bearing
            #plt.pause(0.01)

    return measurement_matrix

def calculate_likelihood(zk, zk_predicted):
    global range_error
    R = np.diag([range_error, range_error, range_error, range_error, range_error]).reshape(5,5)
    zk = np.ndarray.tolist(zk.flatten())
    likelihood_rv = multivariate_normal(mean=zk, cov=R)
    likelihood_array = []
    for particle in range(zk_predicted.shape[1]):
        #lk = likelihood_rv.pdf(x=zk_predicted[:, particle])
        lk = np.sum(np.abs(zk_predicted[:, particle] - zk))**2
        if lk == 0:
            lk = 10.0
        else:
            lk = 1/lk
        likelihood_array.append(lk)

    return likelihood_array

def resample(Xkk, wkk):
    idx_max = np.argmax(wkk.reshape(-1))
    Xkk_max = Xkk[:, idx_max]
    n_particle = Xkk.shape[1] #columns are particles
    idx = np.random.choice(n_particle, n_particle, replace=True, p=wkk.reshape(-1))

    Q = Qk
    # x, y, V, a, heading, phi(w)
    Q = np.diag(Q)
    n_state_dims = Xkk.shape[0]
    resample_rv = multivariate_normal(mean=np.zeros(n_state_dims), cov=Q)
    Xkk = Xkk[:, idx] + resample_rv.rvs(size=n_particle).T
    Xkk[2, :] = np.abs(Xkk[2, :])
    Xkk[2, :] = np.where(Xkk[2, :] < 0.1, Xkk[2, :], 0.1) # map Velocity range to: (min_Velocity, max_Velocity)
    Xkk[3, :] = np.abs(Xkk[3, :])
    Xkk[3, :] = np.where(Xkk[3, :] < 0.01, Xkk[3, :], 0.01) # map Acceleration range to: (min_Acceleration, max_Acceleration)
    Xkk[:, 0] = Xkk_max
    wkk[:] = 1 / n_particle
    return Xkk, wkk
    
def estimate_target_state(Xkk, wkk, lk):

    max_idx = np.argmax(lk)
    Xkk_MAP = Xkk[:,max_idx]
    
    Xkk_MMSE = np.matmul(Xkk, wkk)

    headings = Xkk[4, :].reshape(-1)
    while np.any(np.abs(headings - Xkk_MMSE[4]) > np.pi):
        Xkk_MMSE[4], headings = adjust_angles(Xkk_MMSE[4], headings)
        Xkk_MMSE[4] = np.mean(headings)
    
    return Xkk_MAP, Xkk_MMSE
