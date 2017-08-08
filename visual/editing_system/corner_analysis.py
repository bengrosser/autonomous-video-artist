#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : corner_analysis.py
 #Creation Date : 03-08-2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.

'''
This is an analysis script for all the ellipse videos to 
find editing point as part of autonomous post analysis library
'''

import cv2
import numpy as np
import time

def xor_analysis(frame_1, frame_2):
    return 

    

def and_analysis(frame_1, frame_2):
    return 


def ellipses_analysis(video_name_1, video_name_2, option=0):
    #Grab frames from each of the two videos, and compare each
    #of them

    camera_1 = cv2.VideoCapture(video_name_1)
    camera_2 = cv2.VideoCapture(video_name_2)
    start_time = time.time()
    while True:
        grabbed_1, frame_1 = camera_1.read()
        if grabbed_1:
            while True:
                grabbed_2, frame_2 = camera_2.read()
                if grabbed_2:

                    
                





        else:
            print("No video feed available")
            break
    end_time = time.time()
    print end_time - start_time 
    camera.release()
    out.release()
    cv2.destroyAllWindows()


