#!/usr/bin/env python
"""
Created on Mon Aug 24 15:12:36 2020

@author: JHP
"""

import numpy as np
from math import sin, cos, tan, copysign, sqrt, degrees, pi
from scipy.spatial import distance
import os, sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import mathdir.cubic_spline_planner as cubic_spline_planner
import mathdir.cartesian_frenet_conversion as cartesian_frenet_conversion
import time
import bisect

BASE_ITER = 30

class GlobalPath:
    def __init__(self, filename = '/home/macaron/catkin_ws/src/macaron_2/path/manhae1.npy', x = [], y = []):
        if len(x) > 0:
            self.rx, self.ry, self.ryaw, self.rk, self.rdk, self.s = cubic_spline_planner.calc_spline_course(x, y)
        else :
            pathArray = np.load(file= filename)
            gx = pathArray[0:pathArray.shape[0]-1, 0]
            gy = pathArray[0:pathArray.shape[0]-1, 1]
            self.rx, self.ry, self.ryaw, self.rk, self.rdk, self.s = cubic_spline_planner.calc_spline_course(gx, gy)
        self.cur_ref_index = 0
        self.cur_s_ref_index = 0
        self.last_search_time = 0

    def getClosestSIndexCurXY(self, x, y, mode):
        iteration = 1
        if mode == 0 :
           cur_time = time.time()
           time_elapsed = len(self.rx) if cur_time - self.last_search_time > len(self.rx) else cur_time - self.last_search_time
           iteration = int(time_elapsed + 1) * BASE_ITER
           self.last_search_time = cur_time
        if mode == 1:
           iteration = len(self.rx)
        #print("time",int(cur_time - self.last_search_time) + 1)
        ref_index = cartesian_frenet_conversion.getClosestSPoint(self.rx, self.ry, x, y, self.cur_ref_index, iteration)
        if mode == 0:
            self.cur_ref_index = ref_index
        return ref_index

    def getClosestSIndexCurS(self, s):
        return bisect.bisect(self.s, s) - 1
        
    # mode 0 -> 찾던 위치 근처에서 찾기, mode 1 처음부터 찾기
    def xy2sl(self,x, y, mode = 0):
        ref_index = self.getClosestSIndexCurXY(x, y, mode)
        self.cur_ref_index = ref_index
        return self.s[ref_index], cartesian_frenet_conversion.calcOffsetPoint(x, y, self.rx[ref_index], self.ry[ref_index], self.ryaw[ref_index])

    def get_current_reference_point(self):
        return self.rx[self.cur_ref_index], self.ry[self.cur_ref_index], self.ryaw[self.cur_ref_index], self.rk[self.cur_ref_index]
    
    def get_current_reference_yaw(self):
        return self.ryaw[self.cur_s_ref_index]

    def get_current_reference_kappa(self):
        return self.rk[self.cur_s_ref_index]

    def sl2xy(self, s, l):
        ref_index = self.getClosestSIndexCurS(s)
        self.cur_s_ref_index = ref_index
        return cartesian_frenet_conversion.sl2xy(s, l, self.rx[ref_index], self.ry[ref_index], self.ryaw[ref_index])
    
    def getPathFromTo(self, pos1, pos2):
        index1 = self.getClosestSIndexCurXY(pos1[0], pos1[1], 1)
        index2 = self.getClosestSIndexCurXY(pos2[0], pos2[1], 1)
        print(index1,index2)
        return self.rx[index1:index2], self.ry[index1:index2] 
            
        



def main():
    import rospy
    import matplotlib.pyplot as plt
    import sensor.sensor_data_communication as sensor_data_communication

    rospy.init_node("test_converter")
    dataHub = sensor_data_communication.sensor_data_communicationer()

    testPath = GlobalPath()
    
    x, y, yaw = [], [], []
    a = True
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        while not dataHub.readySensor():
            #print("Sensor is not ready")
            continue
        if a:
            print("start!")
            a = False
        x, y, yaw = dataHub.get_pose()
        print("ori",x,y)
        #refpoint = converter.getClosestSPoint(x, y)
        s, l = testPath.xy2sl(x,y)
        print("s, l", s, l)
        re_x, re_y = testPath.sl2xy(s, l)
        print("x, y", re_x, re_y)
        dataHub.setSensorFlagOff()
        rate.sleep()


if __name__ == '__main__':
    main()
    

    