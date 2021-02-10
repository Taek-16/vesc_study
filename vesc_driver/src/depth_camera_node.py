#!/usr/bin/env python

import numpy as np
import cv2
import threading
from depth_camera import real_sense

import os, sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import time

class depth_camera_saver :
    def __init__(self, folder_path):
        self.folder_path = str(folder_path)
        self.is_ready = False
        
    def notify(self, rgb_img, depth_img):
        self.is_ready = True
        current_time = time.strftime('%Y_%m_%d_%Hh%Mm%Ss', time.localtime(time.time()))
        
        cv2.imwrite("./"+self.folder_path + "/" +str(current_time)+"_rgb.png", rgb_img)
        cv2.imwrite("./"+self.folder_path + "/" +str(current_time)+"_depth.png", depth_img)

def createFolder(dirName):
    try:
        if not os.path.exists('./'+dirName):
            os.makedirs('./'+dirName)
    except OSError:
        print ('Error: Creating directory. ' +  dirName)
        
def main():
    rospy.init_node("camera_node")

    savingTimestamp = time.strftime('%Y_%m_%d_%Hh%Mm%Ss', time.localtime(time.time()))
    createFolder(savingTimestamp)
    realsense_manager = real_sense()
    img_saver = depth_camera_saver(savingTimestamp)

    realsense_manager.register_observer(img_saver)

    rate = rospy.Rate(10)
    #vesc_input = VescInput()
    realsense_manager.start()
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__' :
    main()
