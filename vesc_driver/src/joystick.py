#!/usr/bin/env python
#-*-coding:utf-8-*-

import pygame
from pygame.locals import *
import threading


class joystick_handler(object):
    def __init__(self, id):
        self.id = id
        self.joy = pygame.joystick.Joystick(id)
        self.name = self.joy.get_name()
        self.joy.init()
        self.numaxes    = self.joy.get_numaxes()
        self.numballs   = self.joy.get_numballs()
        self.numbuttons = self.joy.get_numbuttons()
        self.numhats    = self.joy.get_numhats()

        self.axis = []
        for i in range(self.numaxes):
            self.axis.append(self.joy.get_axis(i))

        self.ball = []
        for i in range(self.numballs):
            self.ball.append(self.joy.get_ball(i))

        self.button = []
        for i in range(self.numbuttons):
            self.button.append(self.joy.get_button(i))

        self.hat = []
        for i in range(self.numhats):
            self.hat.append(self.joy.get_hat(i))

class joystick_manager(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        pygame.init()
        pygame.event.set_blocked((MOUSEMOTION, MOUSEBUTTONUP, MOUSEBUTTONDOWN))
        self.joycount = pygame.joystick.get_count()
        if self.joycount == 0:
            print("No joystick manager detected!")
            self.quit(1)
        self.joy = []
        for i in range(self.joycount):
            self.joy.append(joystick_handler(i))
        self.observers = []
        self.isRun = True

    def register_observer(self, observer):
        self.observers.append(observer)

    def notify_observer(self, event_type, key, value):
        for observer in self.observers:
            observer.notify(event_type, key, value)

    def run(self):
        while self.isRun:
            for event in [pygame.event.wait(), ] + pygame.event.get():
                # QUIT             none
                # ACTIVEEVENT      gain, state
                # KEYDOWN          unicode, key, mod
                # KEYUP            key, mod
                # MOUSEMOTION      pos, rel, buttons
                # MOUSEBUTTONUP    pos, button
                # MOUSEBUTTONDOWN  pos, button
                # JOYAXISMOTION    joy, axis, value
                # JOYBALLMOTION    joy, ball, rel
                # JOYHATMOTION     joy, hat, value
                # JOYBUTTONUP      joy, button
                # JOYBUTTONDOWN    joy, button
                # VIDEORESIZE      size, w, h
                # VIDEOEXPOSE      none
                # USEREVENT        code
                if event.type == KEYDOWN and event.key in [K_ESCAPE, K_q]:
                    self.quit()
                elif event.type == JOYAXISMOTION: 
                    self.notify_observer(event.type, event.axis, event.value)
                elif event.type == JOYHATMOTION:
                    self.notify_observer(event.type, event.hat, event.value)
                elif event.type == JOYBUTTONUP:
                    self.notify_observer(event.type, event.button, 0)
                elif event.type == JOYBUTTONDOWN:
                    self.notify_observer(event.type, event.button, 1)
    
    def quit(self):
        self.isRun = False