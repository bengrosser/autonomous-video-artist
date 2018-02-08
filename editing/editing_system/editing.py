# Scripts for all the editing utilities
# The whole editing process will be divided to tree stages
# Import stage:
#   Grab data from database
#   Or in the test phase, read test videos and json files
#   Build dictionary based on vid_names and corresponding subclips
# Initialization Stage:
#   Compare all the frames in sub-clips imported find the best editing point
#   Build Reference Dictionary
# Editing Stage:
#   ....
import cv2
import numpy as np
import epf
import math
import sqlite3
import time
import json
# TODO: Checkout the generators if the performance really sucks
# TODO: Implement actual import stage methods


# Helper function for test methods in importing stage
def read_frames_into_list(vid_path):
    camera = cv2.VideoCapture(vid_path)
    vid_frames_in_list = []
    while True:
        grabbed, frame = camera.read()
        if grabbed:
            vid_frames_in_list.append(frame)
        else:
            print "No Video Feed Available, Finished Import", vid_path
            break
    camera.release()
    return vid_frames_in_list


# import stage test methods
def test_import(test_files_root_path):
    """
    :param test_files_root_path: root path for all test files
    :return: the dictionary with format {vid_name:{editing_pair: corresponding frames}}
    """
    json_file_path = test_files_root_path + "/vid_subclip.json"
    imported_video_sources = {}
    vid_subclip_info = json.load(open(json_file_path))
    for vid_name, editing_pair in vid_subclip_info.iteritems():
        editing_pair = (editing_pair[0], editing_pair[1])
        vid_path = test_files_root_path + "/" + vid_name
        frames_list = read_frames_into_list(vid_path)
        imported_video_sources[vid_name] = {editing_pair: frames_list}
    return imported_video_sources


# Initialization stage
#   Compare all the frames in sub-clips imported find the best editing point
#   Build Reference Dictionary: this step is done inside editing point finder
# The reference dictionary should contain three top layers corresponding to DA methods
def initialize(imported_video_sources):












































