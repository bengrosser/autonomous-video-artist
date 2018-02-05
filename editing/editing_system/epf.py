# Scripts for editing point finder
# EPF will try to find frames in videos that have the most
# logic compatibility
# The vector it produces will also be used to construct edges

import cv2
import numpy as np
import gradient
import math
import time
import sqlite3


# Calculate the image gradient compatibility
def compare_gradient(img1, img2):
    """
    :param img1: The frame read from first video
    :param img2: The frame read from second video
    :return: a vector with both magnitude and direction
    """
    img1 = cv2.cvtColor(img1, cv2.COLOR_BRG2GRAY)
    img1_blur = gradient.gaussian_blur(img1)
    img2 = cv2.cvtColor(img2, cv2.COLOR_BRG2GRAY)
    img2_blur = gradient.gaussian_blur(img2)
    img1_sobelx, img1_sobely = gradient.sobel_filter(img1_blur)
    img2_sobelx, img2_sobely = gradient.sobel_filter(img2_blur)
    img1_gradient_intensity, img1_raw_direction_indegree = gradient.get_gradients(img1_sobelx, img1_sobely)
    img2_gradient_intensity, img2_raw_direction_indegree = gradient.get_gradients(img2_sobelx, img2_sobely)
    return generate_similarity_vector(img1_gradient_intensity, img2_gradient_intensity,
                                      img1_raw_direction_indegree, img2_raw_direction_indegree)


# Based on both images' edge vectors' directions and intensity we can have a compatibility vector
def generate_similarity_vector(intensity_1, intensity_2, direction_1, direction_2):
    positive_45_unit_value = 1/math.sqrt(2)
    negative_45_unit_value = -1/math.sqrt(2)
    dir_map = {"90": np.array([1, 0]), "-90": np.array([-1, 0]), "1": np.array([0, 1]), "-1": np.array([0, -1]),
               "-45": np.array([negative_45_unit_value, negative_45_unit_value]),
               "45": np.array([positive_45_unit_value, positive_45_unit_value])}
    rows_num, cols_num = intensity_1.shape
    similarity_vector = np.array([0, 0])
    for i in rows_num:
        for j in cols_num:
            unit_vector_1 = dir_map[str(direction_1[i][j])]
            unit_vector_2 = dir_map[str(direction_2[i][j])]
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


def gradient_epf(vid1_edit_frame_dict, vid2_edit_frame_dict):
    vid1_lowest_key = None
    vid2_lowest_key = None
    vid1_frame_offset = 0
    vid2_frame_offset = 0
    lowest_vector = None
    lowest_magnitude = float('inf')
    vid1_counter = 0
    for key_1, frame_1 in vid1_edit_frame_dict.iteritems():
        vid2_counter = 0
        for key_2, frame_2 in vid2_edit_frame_dict.iteritems():
            similarity_vector = compare_gradient(frame_1, frame_2)
            similarity_vector_magnitude = vector_magnitude(similarity_vector)
            if similarity_vector_magnitude < lowest_magnitude:
                vid1_lowest_key = key_1
                vid2_lowest_key = key_2
                vid1_frame_offset = vid1_counter
                vid2_frame_offset = vid2_counter
                lowest_vector = similarity_vector
                lowest_magnitude = similarity_vector_magnitude
            vid2_counter += 1
        vid1_counter += 1
    return vid1_lowest_key, vid2_lowest_key, vid1_frame_offset, vid2_frame_offset, lowest_vector


def editing_point_finder(vid1, vid2):
    """
    :param vid1: name of the video 1
    :param vid2: name of the video 2
    :return: the pair of editing points and its corresponding methods
    """
    connection = sqlite3.connect('Video_Metadata.db')
    cursor = connection.cursor()
    cursor.execute('SELECT to_edit, file_path FROM Metadata WHERE file_name=?', (vid1, ))
    connection.close()
    vid1_to_edit, vid1_path = cursor.fetchone()
    cursor.execute('SELECT to_edit, file_path FROM Metadata WHERE file_name=?', (vid2, ))
    vid2_to_edit, vid2_path = cursor.fetchone()
    vid1_edit_frame_dict = build_edit_frame_dict(vid1_path, vid1_to_edit)
    vid2_edit_frame_dict = build_edit_frame_dict(vid2_path, vid2_to_edit)
    vid1_lowest_key, vid2_lowest_key, vid1_frame_offset, vid2_frame_offset, lowest_vector = gradient_epf(vid1_edit_frame_dict, vid2_edit_frame_dict)
    print vid1_lowest_key, vid2_lowest_key, vid1_frame_offset, vid2_frame_offset, lowest_vector





editing_point_finder('2017-10-24_20:1:53.avi', '2017-10-24_20:1:53.avi')






















