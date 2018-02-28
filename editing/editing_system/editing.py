# Scripts for all the editing utilities
# The whole editing process will be divided to tree stages
# Import stage:
#   Grab data from database
#   Or in the test phase, read test videos and json files
#   Change, -> cluster the frames so that there are fewer things to compare in the build of dictionary
#   ----------Build dictionary based on vid_names and corresponding subclips<<<--
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
import cPickle


# TODO: Checkout the generator if the performance really sucks
# TODO: Implement actual import stage methods ---> gradient epf


# Get the frame sequential generator for each of the video source
def load_generator(vid_path):
    camera = cv2.VideoCapture(vid_path)
    return camera


# Import stage using frame generator to save memory
def test_import_generator(test_files_root_path):
    """
    :param test_files_root_path: root path for all test files
    :return: the dictionary with format {vid_name: [[editing_pairs], generator]}
    """
    json_file_path = test_files_root_path + "/vid_subclip.json"
    imported_video_sources = {}
    vid_subclip_info = json.load(open(json_file_path))
    for vid_name, editing_pair in vid_subclip_info.iteritems():
        print "Start to work on ", vid_name
        editing_pair = (editing_pair[0], editing_pair[1])
        vid_path = test_files_root_path + "/" + vid_name
        vid_generator = load_generator(vid_path)
        imported_video_sources[vid_name] = [[editing_pair], vid_generator]
    return imported_video_sources


# Helper function for test methods in importing stage
def read_frames_into_list(vid_path):
    camera = cv2.VideoCapture(vid_path)
    vid_frames_in_list = []
    while True:
        grabbed, frame = camera.read()
        if grabbed:
            vid_frames_in_list.append(frame)
        else:
            print "Finished Import", vid_path
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
        print "Start to work on ", vid_name
        editing_pair = (editing_pair[0], editing_pair[1])
        vid_path = test_files_root_path + "/" + vid_name
        frames_list = read_frames_into_list(vid_path)
        imported_video_sources[vid_name] = {editing_pair: frames_list}
    return imported_video_sources


# Methods with all frames loaded to memory
# Initialization stage
#   Compare all the frames in sub-clips imported find the best editing point
#   Build Reference Dictionary: this step is done inside editing point finder
# The reference dictionary should contain three top layers corresponding to DA methods
def initialize(imported_video_sources, ff_memory_to_update):
    """
    :param imported_video_sources: the dictionary with format {vid_name:{editing_pair: corresponding frames}}
    :param ff_memory_to_update: used to book keeping comparison results
    :return: lowest vector coordinate, ff_memory
    """
    vid_names = imported_video_sources.keys()
    num_of_videos = len(vid_names)
    lowest_vector_vid1_key = None
    lowest_vector_vid2_key = None
    lowest_vector_vid1_pair_key = None
    lowest_vector_vid2_pair_key = None
    lowest_vector_vid1_offset = None
    lowest_vector_vid2_offset = None
    lowest_vector = None
    lowest_magnitude = float('inf')
    for i in range(num_of_videos):
        for j in range(i + 1, num_of_videos):
            print "Start to work on", vid_names[i], vid_names[j]
            start_time = time.time()
            local_lowest_pair_key_1, local_lowest_pair_key_2, local_vid1_cluster_offset, local_vid2_cluster_offset, \
            local_lowest_vector, local_lowest_magnitude, ff_memory_to_update = \
                epf.gradient_epf(vid_names[i], imported_video_sources[vid_names[i]], vid_names[j],
                                 imported_video_sources[vid_names[j]], ff_memory_to_update)
            print "One pair spends" + time.time()-start_time, "seconds"
            if local_lowest_magnitude < lowest_magnitude:
                lowest_vector_vid1_key = vid_names[i]
                lowest_vector_vid2_key = vid_names[j]
                lowest_vector_vid1_pair_key = local_lowest_pair_key_1
                lowest_vector_vid2_pair_key = local_lowest_pair_key_2
                lowest_vector_vid1_offset = local_vid1_cluster_offset
                lowest_vector_vid2_offset = local_vid2_cluster_offset
                lowest_vector = local_lowest_vector
                lowest_magnitude = local_lowest_magnitude
    return lowest_vector_vid1_key, lowest_vector_vid2_key, lowest_vector_vid1_pair_key, lowest_vector_vid2_pair_key, \
           lowest_vector_vid1_offset, lowest_vector_vid2_offset, lowest_vector, ff_memory_to_update


# Methods using generators
# Initialization stage
#   Compare all the frames in sub-clips imported find the best editing point
#   Build Reference Dictionary: this step is done inside editing point finder
# The reference dictionary should contain three top layers corresponding to DA methods
def generator_initialize(imported_video_sources, ff_memory_to_update):
    """
    :param imported_video_sources: the dictionary with format {vid_name: [[editing pairs], video generator]]}
    :param ff_memory_to_update: used to book keeping comparison results
    :return: lowest vector coordinate, ff_memory
    """
    vid_names = imported_video_sources.keys()
    num_of_videos = len(vid_names)
    lowest_vector_vid1_key = None
    lowest_vector_vid2_key = None
    lowest_vector_vid1_pair_key = None
    lowest_vector_vid2_pair_key = None
    lowest_vector_vid1_offset = None
    lowest_vector_vid2_offset = None
    lowest_vector = None
    lowest_magnitude = float('inf')
    for i in range(num_of_videos):
        for j in range(i + 1, num_of_videos):
            print "Start to work on", vid_names[i], vid_names[j]
            start_time = time.time()
            local_lowest_pair_key_1, local_lowest_pair_key_2, local_vid1_cluster_offset, local_vid2_cluster_offset, \
            local_lowest_vector, local_lowest_magnitude, ff_memory_to_update = \
                epf.gradient_epf_with_generators(vid_names[i], imported_video_sources[vid_names[i]], vid_names[j],
                                 imported_video_sources[vid_names[j]], ff_memory_to_update)
            print "Spend", time.time()-start_time, "seconds on pair",vid_names[i],vid_names[j]
            if local_lowest_magnitude < lowest_magnitude:
                lowest_vector_vid1_key = vid_names[i]
                lowest_vector_vid2_key = vid_names[j]
                lowest_vector_vid1_pair_key = local_lowest_pair_key_1
                lowest_vector_vid2_pair_key = local_lowest_pair_key_2
                lowest_vector_vid1_offset = local_vid1_cluster_offset
                lowest_vector_vid2_offset = local_vid2_cluster_offset
                lowest_vector = local_lowest_vector
                lowest_magnitude = local_lowest_magnitude
    return lowest_vector_vid1_key, lowest_vector_vid2_key, lowest_vector_vid1_pair_key, lowest_vector_vid2_pair_key, \
           lowest_vector_vid1_offset, lowest_vector_vid2_offset, lowest_vector, ff_memory_to_update


start_time = time.time()
imported_videos_sources = test_import_generator("./test")
# imported_videos_sources = test_import("./test")
print "Spend", time.time()-start_time, "seconds to import videos"
ff_memory = {}
result_vid1_key, result_vid2_key, result_vid1_pair_key, result_vid2_pair_key, result_vid1_offset, result_vid2_offset, \
    result_vector, ff_memory = generator_initialize(imported_videos_sources, ff_memory)
print "Spend", time.time()-start_time, "seconds to finish first two stages of editing"
print result_vid1_key, result_vid2_key, result_vid1_pair_key, result_vid2_pair_key, result_vid1_offset, \
    result_vid2_offset, result_vector
print "There are", len(ff_memory), "items in ff_memory"
with open('ff_memory.pickle', 'wb') as descriptor:
    cPickle.dump(ff_memory, descriptor, protocol=cPickle.HIGHEST_PROTOCOL)
