import cv2
import numpy as np
import time
import random
import math


def generate_random_starting_points(num_points, total_num_frames, frames_per_sec, vid_length):
    """
    :param num_points: Number of starting points
    :param total_num_frames: total number of frames in the video
    :param frames_per_sec: FPS
    :param vid_length: duration of video clips
    :return: list of starting points
    """
    duration_in_frames = frames_per_sec * vid_length
    half_duration_in_frames = int(0.5 * duration_in_frames)
    starting_points = []
    points_remain = num_points
    while points_remain != 0:
        temp_start_point = random.randint(0, total_num_frames)
        if temp_start_point + duration_in_frames >= total_num_frames:
            pass
        else:
            if not starting_points:
                starting_points.append(temp_start_point)
                points_remain -= 1
            else:
                if all(abs(i - temp_start_point) > half_duration_in_frames for i in starting_points):
                    starting_points.append(temp_start_point)
                    points_remain -= 1
                else:
                    pass
    return sorted(starting_points)


def split_video(vid_numbers, vid_length, video_path):
    camera = cv2.VideoCapture(video_path)
    fourcc = cv2.cv.CV_FOURCC(*'XVID')
    frames_per_second = math.ceil(camera.get(cv2.cv.CV_CAP_PROP_FPS))
    frames_per_video = int(frames_per_second*vid_length)
    total_frame_num = int(camera.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT))
    starting_points = generate_random_starting_points(vid_numbers, total_frame_num, frames_per_second, vid_length)
    for starting_point in starting_points:
        camera.set(1, starting_point)
        vid_name = str(starting_point) + ".mp4"
        out = cv2.VideoWriter(vid_name, fourcc, camera.get(cv2.cv.CV_CAP_PROP_FPS), (1920, 800))
        for i in range(frames_per_video):
            grabbed, frame = camera.read()
            if grabbed:
                out.write(frame)
            else:
                print "There is something wrong with the video source"
                break
        out.release()
    print "Finished the job"

split_video(10, 10, "/Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/src_video/matrix-woman-red.mp4")

