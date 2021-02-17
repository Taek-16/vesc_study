#!/usr/bin/env python
"""
Created on Thu Aug  6 16:58:05 2020

@author: Elitebook 8570w
"""
import numpy as np
from math import sin, cos, tan, copysign, sqrt, degrees, pi
from scipy.spatial import distance
import os, sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))



SEARCH_MAX_ITER = 5

def getClosestSPoint(rx, ry, x, y, last_search, iteration): 
    mindistance = 999
    position = [x, y]
       
    searchFlag = (last_search + iteration) < len(rx) and (last_search - iteration) > 0
    low = last_search - iteration if searchFlag else 0
    upper = last_search + iteration if searchFlag else len(rx)
    closestrefpoint = 0
    #print("low,upper",low,upper)
    for i in range(low,upper-1):
        ref_position = [rx[i], ry[i]]
        t_distance = distance.euclidean(position, ref_position)
        if t_distance < mindistance :
            mindistance = t_distance
            closestrefpoint = i
        else :
            continue
    #print("index",closestrefpoint)
    return closestrefpoint
    
def calcOffsetPoint(x, y, cur_rx, cur_ry, cur_ryaw):
    position = np.array([x, y])
    ref_point = np.array([cur_rx, cur_ry])
    base_vec = np.array([cos(cur_ryaw), sin(cur_ryaw)])
    pose_vec = np.array(position - ref_point)

    return copysign(distance.euclidean(position, ref_point), np.cross(base_vec, pose_vec))

    
def sl2xy(s, l, cur_rx, cur_ry, cur_ryaw):
    x = cur_rx + l*cos(cur_ryaw + pi/2)
    y = cur_ry + l*sin(cur_ryaw + pi/2)

    return x, y
    
""""
def cartesian_to_frenet(self,cur_rs,cur_rx,cur_ry,rtheta,rkappa,rdkappa,x,y,v,a,theta,kappa):
    dx = x - self.rx
    dy = y - self.ry
    d, d_d, d_dd, s, s_d, s_dd = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    cos_theta_r = cos(self.ryaw)
    sin_theta_r = sin(self.ryaw)
        
    cross_rn_nd = cos_theta_r * dy - sin_theta_r * dx
    d = copysign(sqrt(dx*dx+dy*dy), cross_rn_nd)
        
    delta_theta = theta - self.ryaw
    tan_delta_theta = tan(delta_theta)
    cos_delta_theta = cos(delta_theta)
        
    one_minus_kappa_r_d = 1 - self.rk * d
    d_d = one_minus_kappa_r_d * tan_delta_theta
        
    kappa_r_d_prime = self.rdk * d + self.rk * d_d
    d_dd = -1.0 * kappa_r_d_prime * tan_delta_theta + one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta * (kappa * one_minus_kappa_r_d/ cos_delta_theta - self.rk)
        
    s = rs
        
    s_d = v * cos_delta_theta/ one_minus_kappa_r_d
    delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - self.rk
        
    s_dd = (a * cos_delta_theta - s_d * s_d * 
            (d_d * delta_theta_prime - kappa_r_d_prime))/one_minus_kappa_r_d
        
    return s, s_d, s_dd, d, d_d, d_dd
"""

def main():
    import rospy
    import matplotlib.pyplot as plt
    import sensor.sensor_data_communication as sensor_data_communication

    rospy.init_node("test_converter")
    dataHub = sensor_data_communication.sensor_data_communicationer()

    testPath = path_data.Path()
    
    x, y, yaw = [], [], []
    a = True
    while not rospy.is_shutdown():
        while not dataHub.readySensor():
            print("Sensor is not ready")
            continue
        if a:
            print("start!")
            a = False
        x, y, yaw = dataHub.get_pose()
        #refpoint = converter.getClosestSPoint(x, y)
        s, l = testPath.xy2sl(x,y)
        print("s, l", s, l)
        re_x, re_y = testPath.sl2xy(s, l)
        print("x, y", re_x, re_y)
        rate.sleep()


if __name__ == '__main__':
    main()
    