import pickle
import time
import cv2
import gradient
import numpy as np
from editing_structures import EditingBlock, AssembledBlocks
from frame_cluster import cluster_video_frames_intense
from audio_helper import create_cluster_sound


def generate_frame_gradient(frame):
    """
    Generate one gradient DA visualization frame
    :param frame: input frame from original video
    :return: visualized frame
    """
    color_dir_map = np.array([[0,190,200],[200,70,70],[51,204,233],[120,80,220],[220,0,80],[200,200,60]])
    background_color = (0, 0, 0)
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img_blur = gradient.gaussian_blur(img.copy())
    sobelx, sobely = gradient.sobel_filter(img_blur)
    gradient_intensity, raw_direction_indegree = gradient.get_gradients_with_jit(sobelx, sobely)
    result = gradient.visualize(img_blur, gradient_intensity, raw_direction_indegree, color_dir_map, background_color)
    hsv = gradient.change_color(result, gradient_intensity)
    final_result = cv2.cvtColor(hsv , cv2.COLOR_HSV2RGB)
    return final_result


# TODO: Later modify it to accommodate editing range
def generate_video(assemble_blocks, output):
    """
    Generate result video and progress video from assemble_blocks class
    :param assemble_blocks: abstract data result from editing
    :param output: output name for the video
    """
    start_time = time.time()
    fourcc = cv2.cv.CV_FOURCC(*'XVID')
    frame_rate = float(23)
    resolution = (int(1920), int(800))
    visual_bridge_name = "visual_bridge_" + output
    edited_result = cv2.VideoWriter(output, fourcc, frame_rate, resolution)
    # visual_bridge_result = cv2.VideoWriter(visual_bridge_name, fourcc, frame_rate, resolution)
    clustered_videos = {}
    counter = 0
    for editing_block in assemble_blocks.editing_blocks:
        print editing_block
        block_vid_name = editing_block.video_name
        block_cluster_index = editing_block.cluster_index
        if 'her' in block_vid_name:
            block_threshold = 0.1
        else:
            block_threshold = 0.025
        if block_vid_name not in clustered_videos:
            file_name = "./test/" + block_vid_name
            camera = cv2.VideoCapture(file_name)
            cluster_start_time = time.time()
            clustered_video = cluster_video_frames_intense(camera, block_threshold)
            # clustered_video = cluster_video_frames_intense(camera, editing_block.cluster_threshold)
            print "Spend", time.time()-cluster_start_time, "to cluster"
            clustered_videos[block_vid_name] = clustered_video
            cluster_to_use = clustered_video[block_cluster_index]
            num_frames = len(cluster_to_use)
            frame_counter = 0
            for frame in cluster_to_use:
                edited_result.write(frame)
                # if frame_counter > float(num_frames/2) or num_frames == 1:
                #     visualized_frame = generate_frame_gradient(frame)
                #     visual_bridge_result.write(visualized_frame)
                # else:
                #     visual_bridge_result.write(frame)
                frame_counter += 1
        else:
            cluster_to_use = clustered_videos[block_vid_name][block_cluster_index]
            num_frames = len(cluster_to_use)
            frame_counter = 0
            for frame in cluster_to_use:
                edited_result.write(frame)
                # if frame_counter > float(num_frames/2) or num_frames == 1:
                #     visualized_frame = generate_frame_gradient(frame)
                #     visual_bridge_result.write(visualized_frame)
                # else:
                #     visual_bridge_result.write(frame)
                frame_counter += 1
        print "Finished the block"
    print "Spend", time.time()-start_time, "to generate video"


with open("assembled_video.pickle", 'rb') as input_source:
    assembled_blocks = pickle.load(input_source)
    generate_video(assembled_blocks, "her_matrix.mp4")
    # create_cluster_sound(assembled_blocks)
    # print assembled_blocks.editing_blocks[0].video_name
    # print assembled_blocks.editing_blocks[0].cluster_index










