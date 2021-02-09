#!/usr/bin/env python
#-*-coding:utf-8-*-

from pygame.locals import *
import rospy
import os, sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

# ros message
from vesc_msgs.msg import VescInput
from joystick import joystick_manager
import threading

# JOYAXISMOTION, JOYHATMOTION, JOYBUTTONUP, JOYBUTTONDOWN
# JOYAXISMOTION : axisno. 2,  brake, range : -1~1, 
# axisno. 5, speed, range : -1~1,
# axisno. 0, servo_position, -1~1 
# JOYHAT : v[1] => duty_cycle , 1 : 0.1  increase, -1 : 0.1 decrease
class joystick_vesc_msg_creator:
    def __init__(self):
        self.fbrake = 0.0
        self.fspeed = 0.0
        self.fservo_position = 0.0
        self.fduty_cycle = 0.0
        self.is_ready = False
    def notify(self, event_type, key, value):
        self.is_ready = True
        if event_type == JOYAXISMOTION:
            if key == 2 :
                self.fbrake = self.__convert_brake(value)
            elif key == 5 :
                self.fspeed = self.__convert_speed(value)
            elif key == 0 :
                self.fservo_position = self.__convert_servo(value)
        elif event_type == JOYHATMOTION:
            if key == 0 :
                self.fduty_cycle += self.__convert_duty_cycle(value[1])

    def __convert_speed(self, value):
        return (value + 1) * 1000

    def __convert_servo(self, value):
        return (value + 1)/2

    def __convert_duty_cycle(self, value):
        return value * 0.1

    def __convert_brake(self, value):
        return (value + 1)*10

    def get_send_message(self):
        self.is_ready = False
        return self.fbrake, self.fspeed, self.fservo_position, self.fduty_cycle

    def msg_ready(self):
        return self.is_ready

def main():
    rospy.init_node("control_node")
    control_pub = rospy.Publisher("joystick_input", VescInput, queue_size=1)
    joyManager = joystick_manager()
    joyObserver = joystick_vesc_msg_creator()

    joyManager.register_observer(joyObserver)

    rate = rospy.Rate(20)
    vesc_input = VescInput()
    joyManager.start()
    while not rospy.is_shutdown():
        while not joyObserver.msg_ready():
            rate.sleep()
            continue

        vesc_input.write_brake, \
        vesc_input.write_speed, \
        vesc_input.write_servo_position, \
        vesc_input.write_duty_cycle = joyObserver.get_send_message()
        control_pub.publish(vesc_input)
        rate.sleep()

if __name__ == '__main__':
    main()