#!/usr/bin/env python

import pyrealsense2 as rs
import numpy as np
import cv2
import threading

VISUALIZATION = False

class real_sense(threading.Thread):
    def __init__(self, alpha = 0.1, width = 640, height = 480):
        threading.Thread.__init__(self)
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
        self.observers = []

    def register_observer(self, observer):
        self.observers.append(observer)

    def notify_observer(self, rgb_img, depth_img):
        for observer in self.observers:
            observer.notify(rgb_img, depth_img)

    def run(self):

        # Start streaming
        self.pipeline.start(self.config)

        try:
            while True:

                # Wait for a coherent pair of frames: depth and color
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                self.notify_observer(color_image,depth_colormap)

                # Show images
                if VISUALIZATION :
                    # Stack both images horizontally
                    images = np.hstack((color_image, depth_colormap))

                    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                    cv2.imshow('RealSense', images)
                    cv2.waitKey(1)
        finally:

            # Stop streaming
            self.pipeline.stop()

    def quit(self):
        self.pipeline.stop()