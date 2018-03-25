import cv2
import time
import json
from pydub import AudioSegment
from frame_cluster import adaptive_cluster, get_block_time_range


# Test Method, may not be used in the real production
# TODO: Need to have a more accurate method to measure time
def get_sound_segment(json_path, original_file_path):
    """
    Using the stored clip information form original video in json to get the sound
    :param json_path: JSON file path
    :param original_file_path: the file path to the video where all test videos divided from
    """
    original_audio = AudioSegment.from_file(original_file_path, "mp4")
    camera = cv2.VideoCapture(original_file_path)
    (major_ver, minor_ver, subminor_ver, _) = (cv2.__version__).split('.')
    if int(major_ver) < 3:
        fps = camera.get(cv2.cv.CV_CAP_PROP_FPS)
        print "Frames per second using video.get(cv2.cv.CV_CAP_PROP_FPS): {0}".format(fps)
    else:
        fps = camera.get(cv2.CAP_PROP_FPS)
        print "Frames per second using video.get(cv2.CAP_PROP_FPS) : {0}".format(fps)
    vid_subclip_info = json.load(open(json_path))
    for vid_name, editing_pair in vid_subclip_info.iteritems():
        print "Start to work on segment", vid_name, "'s sound"
        mp3_name = './test/' + vid_name.replace("mp4", "mp3")
        start_time = (editing_pair[0]/fps)*1000
        end_time = (editing_pair[1]/fps)*1000
        print "Start time is", start_time
        print "End time is", end_time
        segmented_result = original_audio[start_time:end_time]
        print "This segment has", segmented_result.duration_seconds, "seconds"
        segmented_result.export(mp3_name, format="mp3")
    print "Finished produce sound segments"


# TODO: Add AssembledBlocks structures as parameters
def create_cluster_sound(assembled_blocks):
    """
    Create sounds based on the AssembledBlocks passed in
    :param: assembled blocks
    """
    start_time = time.time()
    final_result_audio = None
    fps = 23.99
    clustered_videos = {}
    for editing_block in assembled_blocks.editing_blocks:
        print editing_block
        block_vid_name = editing_block.video_name
        block_cluster_index = editing_block.cluster_index
        file_name = "./test/" + block_vid_name
        audio_path = file_name.replace("mp4", "mp3")
        if block_vid_name not in clustered_videos:
            camera = cv2.VideoCapture(file_name)
            (major_ver, minor_ver, subminor_ver, _) = (cv2.__version__).split('.')
            if int(major_ver) < 3:
                fps = camera.get(cv2.cv.CV_CAP_PROP_FPS)
                # print "Frames per second using video.get(cv2.cv.CV_CAP_PROP_FPS): {0}".format(fps)
            else:
                fps = camera.get(cv2.CAP_PROP_FPS)
                # print "Frames per second using video.get(cv2.CAP_PROP_FPS) : {0}".format(fps)
            cluster_start_time = time.time()
            clustered_video = adaptive_cluster(camera, editing_block.cluster_threshold)
            print "Spend", time.time()-cluster_start_time, "to cluster"
            clustered_videos[block_vid_name] = clustered_video
            audio_segmentation = AudioSegment.from_file(audio_path, "mp3")
            start_time, end_time = get_block_time_range(clustered_video, block_cluster_index, fps)
            start_time = (start_time/fps)*1000
            end_time = (end_time/fps)*1000
            cluster_audio = audio_segmentation[start_time:end_time]
            if final_result_audio is None:
                final_result_audio = cluster_audio
            else:
                final_result_audio += cluster_audio
        else:
            clustered_video = clustered_videos[block_vid_name]
            audio_segmentation = AudioSegment.from_file(audio_path, "mp3")
            start_time, end_time = get_block_time_range(clustered_video, block_cluster_index, fps)
            start_time = (start_time/fps)*1000
            end_time = (end_time/fps)*1000
            cluster_audio = audio_segmentation[start_time:end_time]
            if final_result_audio is None:
                final_result_audio = cluster_audio
            else:
                final_result_audio += cluster_audio
        print "Finished the block"
    final_result_audio.export("audio_result.mp3", format="mp3")
    print "Spend", time.time()-start_time, "to generate audio"
