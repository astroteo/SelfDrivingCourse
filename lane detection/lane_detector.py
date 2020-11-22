import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import os
from moviepy.editor import VideoFileClip
from IPython.display import HTML
from collections import deque

from helper import *
testOnline=True # true=save the files in your computer

def process_videos():
    #Function that performs the video lane Detection 
    #video processing (ref: http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_gui/py_video_display/py_video_display.html)
    output_dir="video_output/"
    input_dir="test_video"
    create_directory("video_output")
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    fourcc = cv2.VideoWriter_fourcc(*'FMP4')

    for video_name in os.listdir(input_dir):
        cap = cv2.VideoCapture(input_dir+"/"+video_name)
        out = cv2.VideoWriter(output_dir+video_name,fourcc, 20.0, (960,540)) 
        while(cap.isOpened()):
            ret, frame = cap.read()
            if ret==True:
                image_line = process_image_pipeline(frame)
                out.write(image_line)
            else:
                break
        cap.release()
        out.release()

#Video lane detection function
process_videos() 
