#!/usr/bin/env python

from datetime import datetime
import numpy as np
import cv2
import threading
from depth_camera import real_sense

import os, sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import time
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from time import sleep
import rospy

time_stamp = []
waypoint = [[]]
duty_cycle = []
motor_current = []
brake = []
speed = []
motor_position = []
servo_position = []
heading = []
data_dict = {
    "time_stamp" : time_stamp,
    "waypoint" : waypoint,
    "heading" : heading,
    "duty_cycle" : duty_cycle,
    "motor_current" : motor_current,
    "brake" : brake,
    "speed" : speed,
    "motor_position" : motor_position,
    "servo_position" : servo_position
}
import pickle

class depth_camera_saver :
    def __init__(self, folder_path):
        self.folder_path = str(folder_path)
        self.is_ready = False

    def notify(self, rgb_img, depth_img):
        self.is_ready = True
        self.rgb_img = rgb_img
        self.depth_img = depth_img

    def write_img(self, current_time):
        cv2.imwrite("./"+self.folder_path + "/" +str(current_time)+"_rgb.png", self.rgb_img)
        cv2.imwrite("./"+self.folder_path + "/" +str(current_time)+"_depth.png", self.depth_img)

    def ready_data(self):
        return self.is_ready


class state_saver :
    def __init__(self):
        duty_cycle_sub_ = rospy.Subscriber("commands/motor/duty_cycle", Float64, self.duty_callback, queue_size=1)
        current_sub_ = rospy.Subscriber("commands/motor/current", Float64, self.current_callback, queue_size=1)
        brake_sub_ = rospy.Subscriber("commands/motor/brake", Float64, self.brake_callback, queue_size=1)
        speed_sub_ = rospy.Subscriber("commands/motor/speed", Float64, self.speed_callback, queue_size=1)
        position_sub_ = rospy.Subscriber("commands/motor/position", Float64, self.position_callback, queue_size=1)
        servo_sub_ = rospy.Subscriber("commands/servo/position", Float64, self.servo_callback, queue_size=1)
        way_sub = rospy.Subscriber('current_tm', Pose, self.pose_callback, queue_size=1)
        heading_sub = rospy.Subscriber('heading', Float64, self.head_callback, queue_size=1)
        self.waypoint = []
        self.duty, self.current, self.brake, self.speed, self.position, self.servo = 0,0,0,0,0,0
        self.heading = 0

    def duty_callback(self, value):
        self.duty = value.data
    def pose_callback(self, pose):
        self.waypoint = [pose.position.x, pose.position.y]
    def head_callback(self, heading):
        self.heading = heading.data
    def current_callback(self, value):
        self.current = value.data
    def brake_callback(self, value):
        self.brake = value.data
    def speed_callback(self, value):
        self.speed = value.data
    def position_callback(self, value):
        self.position = value.data
    def servo_callback(self, value):
        self.servo = value.data

    def get_all(self):
        return self.duty, self.current, self.brake, self.speed, self.position, self.servo, self.waypoint, self.heading
    def get_waypiont(self):
        return self.waypoint

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
    state_hub = state_saver()
    realsense_manager.register_observer(img_saver)

    rate = rospy.Rate(10)
    #vesc_input = VescInput()
    realsense_manager.start()
    while not rospy.is_shutdown():
        while not img_saver.ready_data():
            rate.sleep()
            continue
        print(img_saver.ready_data())
        time_stamp = datetime.now().strftime('%Y%m%d%Hh%Mm%S.%f')[:-3]

        duty, current, brake, speed, position, servo, waypoint, heading = state_hub.get_all()
        data_dict["time_stamp"].append(time_stamp)
        data_dict["motor_current"].append(current)
        data_dict["duty_cycle"].append(duty)
        data_dict["brake"].append(brake)
        data_dict["speed"].append(speed)
        data_dict["motor_position"].append(position)
        data_dict["servo_position"].append(servo)
        data_dict["heading"].append(heading)
        data_dict["waypoint"].append(waypoint)

        img_saver.write_img(time_stamp)
        with open("./"+str(savingTimestamp)+ "/" +str(savingTimestamp)+".pickle", "wb") as handle:
            pickle.dump(data_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

        rate.sleep()

if __name__ == '__main__' :
    main()
