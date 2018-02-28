import cv2
import numpy as np
from random import randint
from collections import OrderedDict


def calcEntropy(img):
    hist = cv2.calcHist([img],[0],None,[256],[0,256])
    hist = hist.ravel()/hist.sum()
    logs = np.log2(hist+0.00001)
    entropy = -1 * (hist*logs).sum()
    return entropy


def get_entropy_list(video_name):
    camera = cv2.VideoCapture(video_name)
    entropy_dict = {} 
    # Special Notice that the frame number starts with 0
    frame_number = 0
    while True:
        grabbed, frame = camera.read()
        if grabbed:
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            entropy_dict[frame_number] = calcEntropy(img)
            frame_number += 1
            # print harris_result
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break
    camera.release()
    return entropy_dict 


def sort_dictionary(dictionary_to_sort):
    sorted_dict = OrderedDict(sorted(dictionary_to_sort.items(), key=lambda x: x[1]))
    return sorted_dict


def get_range_pair(editing_point, frames_left, total_num_frames):
    frame_num_for_editing_point = randint(1, frames_left)
    # print "Assigned frame number is " + str(frame_num_for_editing_point)
    frames_left = frames_left - frame_num_for_editing_point
    frame_assigned_left = randint(1, frame_num_for_editing_point)
    frame_assigned_right = frame_num_for_editing_point - frame_assigned_left
    begin_point_frame_number = editing_point - frame_assigned_left
    if begin_point_frame_number < 0:
        begin_point_frame_number = 0
    end_point_frame_number = editing_point + frame_assigned_right
    if end_point_frame_number >= total_num_frames:
        end_point_frame_number = total_num_frames - 1
    return begin_point_frame_number, end_point_frame_number, frames_left


def range_finder(video_name, battery_level, percent_to_use):
    """
    :param video_name: the metadata worker will find range and editing point for this video
    :param battery_level: the battery level metadata associated with the video
    :param percent_to_use: the total percent of frames we are gonna grab from this clip
    :return: A bunch of ranges that is gonna be used for video editor
    The editing point is determined by the frame's entropy value
    """
    video_entropy_dict = get_entropy_list(video_name)
    sorted_video_entropy_dict = sort_dictionary(video_entropy_dict)


    total_number_of_frames = len(sorted_video_entropy_dict)
    clip_frame_number = int(len(sorted_video_entropy_dict)*percent_to_use)
    frames_left = clip_frame_number
    range_result = []
    editing_points_list = []
    while frames_left >= 5:
        # print "Frames left are " + str(frames_left)
        # A random integer from 0 to 164 used to determine which side of sorted dictionary we want to grab
        random_side_indicator = randint(0, 162)

        if random_side_indicator > battery_level:
            # Meaning battery_level is small, we should choose from the lower side
            side = -1
        else:
            # Vice Versa
            side = 1

        if side == -1:
            first_entropy_pair = sorted_video_entropy_dict.items()[0]
            editing_point = first_entropy_pair[0]
            editing_points_list.append(editing_point)
            del sorted_video_entropy_dict[editing_point]
            begin_point_frame_number, end_point_frame_number, frames_left = get_range_pair(editing_point, frames_left,
                                                                                           total_number_of_frames)
            range_result.append((begin_point_frame_number, end_point_frame_number))
        else:
            last_entropy_pair = sorted_video_entropy_dict.items()[len(sorted_video_entropy_dict)-1]
            editing_point = last_entropy_pair[0]
            editing_points_list.append(editing_point)
            del sorted_video_entropy_dict[editing_point]
            begin_point_frame_number, end_point_frame_number, frames_left = get_range_pair(editing_point, frames_left,
                                                                                           total_number_of_frames)
            range_result.append((begin_point_frame_number, end_point_frame_number))

    print editing_points_list
    return range_result



# connection = sqlite3.connect('Video_Metadata.db')
# cursor = connection.cursor()
# vid_name = "2017-10-24_20:2:3.avi"
# cursor.execute('SELECT avg_brightness, avg_distance, motion_detected, has_obstacle, battery_level, '
#                'distance_to_dock, RAM_in_use_sys FROM Metadata WHERE file_name=?', (vid_name,))
# vid_metadata_list = cursor.fetchall()
# if len(vid_metadata_list[0]) != 7:
#     raise Exception('Not enough field for ' + vid_name)
# else:
#     avg_brightness = vid_metadata_list[0][0]
#     avg_distance = vid_metadata_list[0][1]
#     motion_detected = vid_metadata_list[0][2]
#     has_obstacle = vid_metadata_list[0][3]
#     battery_level = vid_metadata_list[0][4]
#     distance_to_dock = vid_metadata_list[0][5]
#     RAM_in_use_sys = vid_metadata_list[0][6]
# range_result, edit_lists =  range_finder('./src.avi', 33, 0.8)
# print range_result
# print edit_lists
