import pyrealsense2 as rs
import numpy as np
from threading import Thread

#TODO: add a depth stream and a method for returning depth data
#also see https://github.com/ReikoR/bbr18-software/blob/001TRT/goal_distance/goal_distance.py
#for one way of reading depth data.

class imageCapRS2:
    def commandThread(self):
        while self.running:
            self.frames = self.pipeline.wait_for_frames()
            self.color_frame = self.frames.get_color_frame()
            self.currentFrame = np.asanyarray(self.color_frame.get_data())

    def __init__(self, src=0):
        #create initial variables for use in methods
        self.running = True
        self.depth_image = None
        self.currentFrame = None

        #create and start the pipeline with a color image stream
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
        self.pipeline.start(self.config)

        #initialize the values for the frame related variables
        self.frames = self.pipeline.wait_for_frames()
        self.color_frame = self.frames.get_color_frame()
        self.currentFrame = np.asanyarray(self.color_frame.get_data())
        Thread(name="commandThread", target=self.commandThread).start()


    def getFrame(self):
        return self.currentFrame

    def setStopped(self, stopped):
        self.pipeline.stop()
        self.running = stopped



