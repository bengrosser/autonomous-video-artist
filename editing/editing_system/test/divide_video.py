import cv2
import random
import math
import json


def generate_random_starting_points(num_points, total_num_frames, frames_per_sec, vid_length):
    """
    :param num_points: Number of starting points
    :param total_num_frames: total number of frames in the video
    :param frames_per_sec: FPS
    :param vid_length: duration of video clips in seconds
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
    """
    :param vid_numbers: Number of subclips
    :param vid_length: Length of the video count in seconds
    :param video_path: path to the video
    """
    camera = cv2.VideoCapture(video_path)
    fourcc = cv2.cv.CV_FOURCC(*'XVID')
    frames_per_second = math.ceil(camera.get(cv2.cv.CV_CAP_PROP_FPS))
    frames_per_video = int(frames_per_second*vid_length)
    total_frame_num = int(camera.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT))
    starting_points = generate_random_starting_points(vid_numbers, total_frame_num, frames_per_second, vid_length)
    vid_subclip_json = {}
    for starting_point in starting_points:
        camera.set(1, starting_point)
        vid_name = "her_" + str(starting_point) + ".mp4"
        vid_subclip_json[vid_name] = (starting_point, starting_point + frames_per_video - 1)
        out = cv2.VideoWriter(vid_name, fourcc, camera.get(cv2.cv.CV_CAP_PROP_FPS), (1920, 800))
        for i in range(frames_per_video):
            grabbed, frame = camera.read()
            if grabbed:
                out.write(frame)
            else:
                print "There is something wrong with the video source"
                break
        out.release()
    with open("vid_subclip_her.json", 'w+') as outfile:
        json.dump(vid_subclip_json, outfile)
    outfile.close()
    print "Finished the job"


def combine_json(json_1, json_2, out_name):
    json_1_dict = json.load(open(json_1))
    json_2_dict = json.load(open(json_2))
    result_dict = {}
    result_dict.update(json_1_dict)
    result_dict.update(json_2_dict)
    with open(out_name, 'w+') as outfile:
        json.dump(result_dict, outfile)
    print "Finished Combining"
# combine_json('vid_subclip.json', 'vid_subclip_her.json', 'vid_subclip_her_matrix.json')
# split_video(10, 10, "/Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/src_video/matrix-woman-red.mp4")
# split_video(10, 10, "/Users/frankshammer42/Documents/CS/autonomous-video-artist/editing/editing_system/test/field_test.mp4")
# split_video(5, 10, "/Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/src_video/her.mp4")
