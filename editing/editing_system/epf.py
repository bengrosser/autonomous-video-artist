# Scripts for editing point finder
# EPF will try to find frames in videos that have the most
# logic compatibility
# The vector it produces will also be used to construct edges

import cv2
import numpy as np
import gradient
import math
import time
from numba import jit
from frame_cluster import cluster_video_frames_intense, sample_clustered_frames


# Calculate the image gradient compatibility with jit enabled methods
def compare_gradient(img1, img2):
    """
    :param img1: The frame read from first video
    :param img2: The frame read from second video
    :return: a vector with both magnitude and direction
    """
    # print "Start work on gradient"
    # start_time = time.time()
    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    img1_blur = gradient.gaussian_blur(img1)
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    img2_blur = gradient.gaussian_blur(img2)
    img1_sobelx, img1_sobely = gradient.sobel_filter(img1_blur)
    img2_sobelx, img2_sobely = gradient.sobel_filter(img2_blur)
    img1_gradient_intensity, img1_raw_direction_indegree = gradient.get_gradients_with_jit(img1_sobelx, img1_sobely)
    img2_gradient_intensity, img2_raw_direction_indegree = gradient.get_gradients_with_jit(img2_sobelx, img2_sobely)
    # print "Spend", time.time()-start_time, "seconds in creating gradient source"
    # start_time = time.time()
    result = generate_similarity_vector(img1_gradient_intensity, img2_gradient_intensity,
                                      img1_raw_direction_indegree, img2_raw_direction_indegree)
    # print "Spend", time.time()-start_time, "seconds gradient calculation"
    return result


# Based on both images' edge vectors' directions and intensity we can have a compatibility vector
@jit(nopython=True)
def generate_similarity_vector(intensity_1, intensity_2, direction_1, direction_2):
    positive_45_unit_value = 1/math.sqrt(2)
    negative_45_unit_value = -1/math.sqrt(2)
    # dir_map = {"90": np.array([1, 0]), "-90": np.array([-1, 0]), "1": np.array([0, 1]), "-1": np.array([0, -1]),
    #            "-45": np.array([negative_45_unit_value, negative_45_unit_value]),
    #            "45": np.array([positive_45_unit_value, positive_45_unit_value])}
    rows_num, cols_num = intensity_1.shape
    similarity_vector = np.array([0.0, 0.0])
    for i in range(rows_num):
        for j in range(cols_num):
            if direction_1[i][j] == 90:
                unit_vector_1 = np.array([1.0, 0.0])
            if direction_1[i][j] == -90:
                unit_vector_1 = np.array([-1.0, 0.0])
            if direction_1[i][j] == 1:
                unit_vector_1 = np.array([0.0, 1.0])
            if direction_1[i][j] == -1:
                unit_vector_1 = np.array([0.0, -1.0])
            if direction_1[i][j] == 45:
                unit_vector_1 = np.array([positive_45_unit_value, positive_45_unit_value])
            if direction_1[i][j] == -45:
                unit_vector_1 = np.array([negative_45_unit_value, negative_45_unit_value])
            if direction_2[i][j] == 90:
                unit_vector_2 = np.array([1.0, 0.0])
            if direction_2[i][j] == -90:
                unit_vector_2 = np.array([-1.0, 0.0])
            if direction_2[i][j] == 1:
                unit_vector_2 = np.array([0.0, 1.0])
            if direction_2[i][j] == -1:
                unit_vector_2 = np.array([0.0, -1.0])
            if direction_2[i][j] == 45:
                unit_vector_2 = np.array([positive_45_unit_value, positive_45_unit_value])
            if direction_2[i][j] == -45:
                unit_vector_2 = np.array([negative_45_unit_value, negative_45_unit_value])
            # unit_vector_1 = dir_map[str(int(direction_1[i][j]))]
            # unit_vector_2 = dir_map[str(int(direction_2[i][j]))]
            vector_1_intensity = intensity_1[i][j]
            vector_2_intensity = intensity_2[i][j]
            vector_1 = unit_vector_1 * vector_1_intensity
            vector_2 = unit_vector_2 * vector_2_intensity
            vector_combo = vector_1 + vector_2
            similarity_vector += vector_combo
    return similarity_vector


def in_range(frame_number, range_pair):
    if range_pair[0] <= frame_number <= range_pair[1]:
        return True
    else:
        return False


def build_edit_frame_dict(vid_path, to_edit):
    """
    :param vid_path: The full path to the video
    :param to_edit: the editing pair stored in the database of that video
    :return: dictionary with editing pair as key and its corresponding frames in the video as value
    """
    vid_edit_frame_dict = {}
    for editing_pair in to_edit:
        vid_edit_frame_dict[editing_pair] = []
    counter = 0
    camera = cv2.VideoCapture(vid_path)
    while True:
        grabbed, frame = camera.read()
        if grabbed:
            for editing_pair in to_edit:
                if in_range(counter, editing_pair):
                    vid_edit_frame_dict[editing_pair].append(frame)
            counter += 1
        else:
            print("No Video Feed Available, Video One Finished")
            break
    camera.release()
    return vid_edit_frame_dict


def vector_magnitude(pair):
    return math.sqrt(pair[0]**2 + pair[1]**2)


# Memory inefficient way of comparing frames and build ff_memory
def compare_frame_list_gradient(vid1_name, frame_list1, key_1, vid2_name, frame_list2, key_2, ff_memory):
    lowest_vector = None
    lowest_magnitude = float('inf')
    list1_frame_offset = 0
    list2_frame_offset = 0
    list1_counter = 0
    for frame_1 in frame_list1:
        list2_counter = 0
        for frame_2 in frame_list2:
            start_time = time.time()
            down_scale_frame_1 = cv2.resize(frame_1, (100, 200))
            down_scale_frame_2 = cv2.resize(frame_2, (100, 200))
            similarity_vector = compare_gradient(down_scale_frame_1, down_scale_frame_2)
            # similarity_vector = compare_gradient(frame_1, frame_2)
            print "Gradient Compariosn spend", time.time()-start_time, "seconds"
            similarity_vector_magnitude = vector_magnitude(similarity_vector)
            memory_key = (vid1_name, vid2_name, key_1, key_2, list1_counter, list2_counter)
            if memory_key in ff_memory:
                ff_memory[memory_key]['gradient'] = similarity_vector
            else:
                ff_memory[memory_key] = {}
                ff_memory[memory_key]['gradient'] = similarity_vector
            if similarity_vector_magnitude < lowest_magnitude:
                list1_frame_offset = list1_counter
                list2_frame_offset = list2_counter
                lowest_vector = similarity_vector
                lowest_magnitude = similarity_vector_magnitude
            list2_counter += 1
        list1_counter += 1
    return lowest_vector, lowest_magnitude, list1_frame_offset, list2_frame_offset, ff_memory


# Using clustered frames to synthesize the contents, more efficient, also used generator
# Threshold value currently is set to 0.025 and it is part of memory key
# TODO: Change this function so that generator just read from keys change the ending condition --> another loop
def compare_frame_cluster(vid1_name, vid1_generator, key_1, vid2_name, vid2_generator, key_2, ff_memory):
    lowest_vector = None
    lowest_magnitude = float('inf')
    list1_cluster_offset = 0
    list2_cluster_offset = 0
    # Preemptive, reset both of the generator
    threshold = 0.025
    clustering_start_time = time.time()
    vid1_generator.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)
    vid2_generator.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)
    clustered_vid1 = cluster_video_frames_intense(vid1_generator, threshold)
    clustered_vid2 = cluster_video_frames_intense(vid2_generator, threshold)
    sampled_frames_cluster1 = sample_clustered_frames(clustered_vid1)
    sampled_frames_cluster2 = sample_clustered_frames(clustered_vid2)
    vid1_generator.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)
    vid2_generator.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)
    # print "Spent", time.time()-clustering_start_time, "seconds to cluster"
    # Start processing the frames
    # print "Length of the sampled frames are", len(sampled_frames_cluster1), len(sampled_frames_cluster2)
    # print "Need to evaluate", len(sampled_frames_cluster1)*len(sampled_frames_cluster2), "pairs"
    for cluster_1_index in range(len(sampled_frames_cluster1)):
        cluster_frame_1 = sampled_frames_cluster1[cluster_1_index]
        for cluster_2_index in range(len(sampled_frames_cluster2)):
            cluster_frame_2 = sampled_frames_cluster2[cluster_2_index]
            down_scale_frame_1 = cv2.resize(cluster_frame_1, (200, 480))
            down_scale_frame_2 = cv2.resize(cluster_frame_2, (200, 480))
            # start_time = time.time()
            similarity_vector = compare_gradient(down_scale_frame_1, down_scale_frame_2)
            # print "Gradient comparison spend", time.time()-start_time, "seconds"
            similarity_vector_magnitude = vector_magnitude(similarity_vector)
            memory_key = (vid1_name, vid2_name, key_1, key_2, cluster_1_index, cluster_2_index, threshold)
            if memory_key in ff_memory:
                ff_memory[memory_key]['gradient'] = similarity_vector
            else:
                ff_memory[memory_key] = {}
                ff_memory[memory_key]['gradient'] = similarity_vector
            if similarity_vector_magnitude < lowest_magnitude:
                list1_cluster_offset = cluster_1_index
                list2_cluster_offset = cluster_2_index
                lowest_vector = similarity_vector
                lowest_magnitude = similarity_vector_magnitude
    vid1_generator.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)
    vid2_generator.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)
    return lowest_vector, lowest_magnitude, list1_cluster_offset, list2_cluster_offset, ff_memory


# Use generator of the video frames to compare gradients instead of using memory
# TODO: Remember to reset the camera/generator back to first frame
# TODO: Change this function so that generator just read from keys change the ending condition
def compare_frame_generator_gradient(vid1_name, vid1_generator, key_1, vid2_name, vid2_generator, key_2, ff_memory):
    lowest_vector = None
    lowest_magnitude = float('inf')
    list1_frame_offset = 0
    list2_frame_offset = 0
    list1_counter = 0
    # Preemptive, reset both of the generator
    vid1_generator.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)
    vid2_generator.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)
    while True:
        grabbed_1, frame_1 = vid1_generator.read()
        if grabbed_1:
            list2_counter = 0
            # Read from video 2
            start_time = time.time()
            while True:
                grabbed_2, frame_2 = vid2_generator.read()
                if grabbed_2:
                    start_time = time.time()
                    down_scale_frame_1 = cv2.resize(frame_1, (200, 480))
                    down_scale_frame_2 = cv2.resize(frame_2, (200, 480))
                    similarity_vector = compare_gradient(down_scale_frame_1, down_scale_frame_2)
                    # similarity_vector = compare_gradient(frame_1, frame_2)
                    print "Gradient comparison spend", time.time()-start_time, "seconds"
                    similarity_vector_magnitude = vector_magnitude(similarity_vector)
                    memory_key = (vid1_name, vid2_name, key_1, key_2, list1_counter, list2_counter)
                    if memory_key in ff_memory:
                        ff_memory[memory_key]['gradient'] = similarity_vector
                    else:
                        ff_memory[memory_key] = {}
                        ff_memory[memory_key]['gradient'] = similarity_vector
                    if similarity_vector_magnitude < lowest_magnitude:
                        list1_frame_offset = list1_counter
                        list2_frame_offset = list2_counter
                        lowest_vector = similarity_vector
                        lowest_magnitude = similarity_vector_magnitude
                    list2_counter += 1
                else:
                    print "Finished reading video 2, spends", time.time()-start_time, "seconds"
                    start_time = time.time()
                    vid2_generator.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)
                    break
            list1_counter += 1
        else:
            print "Finished reading video 1"
            break
    vid1_generator.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)
    vid2_generator.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)
    return lowest_vector, lowest_magnitude, list1_frame_offset, list2_frame_offset, ff_memory


# Methods with all the frames loaded into memory
def gradient_epf(vid1_name, vid1_edit_frame_dict, vid2_name, vid2_edit_frame_dict, ff_memory):
    """
    :param vid1_name: parameter for making ff_memory key
    :param vid1_edit_frame_dict: will be used for calculations
    :param vid2_name: parameter for making ff_memory key
    :param vid2_edit_frame_dict: will be used for calculations
    :param ff_memory: ff_memory stands for frames-frames memory, it is used for book keeping/storing comparison results
    :return: the lowest vector and its coordinate along with updated ff_memory for gradient
    """
    vid1_lowest_key = None
    vid2_lowest_key = None
    vid1_frame_offset = 0
    vid2_frame_offset = 0
    lowest_vector = None
    lowest_magnitude = float('inf')
    for key_1, frame_1_list in vid1_edit_frame_dict.iteritems():
        for key_2, frame_2_list in vid2_edit_frame_dict.iteritems():
            # Compare gradient value in the list
            lowest_similarity_vector, no_use, frame_1_list_offset, frame_2_list_offset, ff_memory = \
                compare_frame_list_gradient(vid1_name, frame_1_list, key_1, vid2_name, frame_2_list, key_2, ff_memory)
            similarity_vector_magnitude = vector_magnitude(lowest_similarity_vector)
            if similarity_vector_magnitude < lowest_magnitude:
                vid1_lowest_key = key_1
                vid2_lowest_key = key_2
                vid1_frame_offset = frame_1_list_offset
                vid2_frame_offset = frame_2_list_offset
                lowest_vector = lowest_similarity_vector
                lowest_magnitude = similarity_vector_magnitude
    return vid1_lowest_key, vid2_lowest_key, vid1_frame_offset, vid2_frame_offset, lowest_vector, lowest_magnitude, ff_memory


# Methods using frame generator
# TODO: write the actual comparison loop, the current one is just for testing purpose, the final version should reset generator based on editing pair
def gradient_epf_with_generators(vid1_name, vid1_edit_source, vid2_name, vid2_edit_source, ff_memory):
    """
    :param vid1_name: parameter for making ff_memory key
    :param vid1_edit_source: [[vid1 editing pairs], vid1 generator]
    :param vid2_name: parameter for making ff_memory key
    :param vid2_edit_source: [[vid2 editing pairs], vid2_generator]
    :param ff_memory: ff_memory stands for frames-frames memory, it is used for book keeping/storing comparison results
    :return: the lowest vector and its coordinate along with updated ff_memory for gradient
    """
    vid1_lowest_key = None
    vid2_lowest_key = None
    vid1_frame_offset = 0
    vid2_frame_offset = 0
    lowest_vector = None
    lowest_magnitude = float('inf')
    vid1_generator = vid1_edit_source[1]
    vid2_generator = vid2_edit_source[1]
    for vid1_editing_pair in vid1_edit_source[0]:
        for vid2_editing_pair in vid2_edit_source[0]:
            # Compare gradient value in the list
            lowest_similarity_vector, no_use, frame_1_list_offset, frame_2_list_offset, ff_memory = \
                compare_frame_cluster(vid1_name, vid1_generator, vid1_editing_pair, vid2_name, vid2_generator, vid2_editing_pair, ff_memory)
            similarity_vector_magnitude = vector_magnitude(lowest_similarity_vector)
            if similarity_vector_magnitude < lowest_magnitude:
                vid1_lowest_key = vid1_editing_pair
                vid2_lowest_key = vid2_editing_pair
                vid1_frame_offset = frame_1_list_offset
                vid2_frame_offset = frame_2_list_offset
                lowest_vector = lowest_similarity_vector
                lowest_magnitude = similarity_vector_magnitude
    return vid1_lowest_key, vid2_lowest_key, vid1_frame_offset, vid2_frame_offset, lowest_vector, lowest_magnitude, ff_memory


# def editing_point_finder(vid1, vid2):
#     """
#     :param vid1: name of the video 1
#     :param vid2: name of the video 2
#     :return: the pair of editing points and its corresponding methods
#     """
#     connection = sqlite3.connect('Video_Metadata.db')
#     cursor = connection.cursor()
#     cursor.execute('SELECT to_edit, file_path FROM Metadata WHERE file_name=?', (vid1, ))
#     connection.close()
#     vid1_to_edit, vid1_path = cursor.fetchone()
#     cursor.execute('SELECT to_edit, file_path FROM Metadata WHERE file_name=?', (vid2, ))
#     vid2_to_edit, vid2_path = cursor.fetchone()
#     vid1_edit_frame_dict = build_edit_frame_dict(vid1_path, vid1_to_edit)
#     vid2_edit_frame_dict = build_edit_frame_dict(vid2_path, vid2_to_edit)
#     vid1_lowest_key, vid2_lowest_key, vid1_frame_offset, vid2_frame_offset, lowest_vector, lowest_magnitude = \
#         gradient_epf(vid1_edit_frame_dict, vid2_edit_frame_dict)
#     return vid1_lowest_key, vid2_lowest_key, vid1_frame_offset, vid2_frame_offset, lowest_vector, lowest_magnitude
