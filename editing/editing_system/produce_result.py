import pickle
import time
import cv2
import gradient
import wave
import ellipse
import numpy as np
from editing_structures import EditingBlock, AssembledBlocks
from frame_cluster import cluster_with_threshold
# Code for generating video editing result from the generated editing structures


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


def generate_frame_ellipse(frame):
    """
    Generate ellipse image for current frame
    :param frame: the frame read from video
    :return: generated ellipse image
    """
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    x,y = img.shape
    harris_result = ellipse.harris_visual(img)
    buf = harris_result.get_data()
    temp_result_matrix = np.frombuffer(buf, np.uint8)
    temp_result_matrix.shape = (x,y,4)
    result_matrix = temp_result_matrix[:,:,0:3]
    return result_matrix


# TODO: Do viewing and screen transformation to produce 3D partitcle graph
# TODO: Generate the video results based upon the editing range
def generate_frame_wave(frame):
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    f = np.fft.fft2(img)
    fshift = np.fft.fftshift(f)
    magnitude_spectrum = 20 * np.log(np.abs(fshift))
    magnitude_spectrum = np.rint(magnitude_spectrum)
    result = np.uint8(magnitude_spectrum)
    result = cv2.cvtColor(result, cv2.COLOR_GRAY2RGB)
    return result


def generate_cut_images(assemble_blocks):
    """
    Generate cut images between two editing blocks
    :param assemble_blocks: abstract data result from editing
    """
    start_time = time.time()
    # visual_bridge_result = cv2.VideoWriter(visual_bridge_name, fourcc, frame_rate, resolution)
    clustered_videos = {}
    counter = 0
    for editing_block in assemble_blocks.editing_blocks:
        # print editing_block
        block_vid_name = editing_block.video_name
        block_cluster_index = editing_block.cluster_index
        if 'her' in block_vid_name:
            block_threshold = 0.1
        else:
            block_threshold = 0.025
        begin_image_name = "./cut_images/" + "block_" + str(counter) + "_begin.jpg"
        end_image_name = "./cut_images/" + "block_" + str(counter) + "_end.jpg"
        if block_vid_name not in clustered_videos:
            file_name = "./test/" + block_vid_name
            camera = cv2.VideoCapture(file_name)
            # cluster_start_time = time.time()
            clustered_video = cluster_with_threshold(camera, block_threshold)
            # clustered_video = adaptive_cluster(camera, editing_block.cluster_threshold)
            # print "Spend", time.time()-cluster_start_time, "to cluster"
            clustered_videos[block_vid_name] = clustered_video
            cluster_to_use = clustered_video[block_cluster_index]
            begin_image = cluster_to_use[0]
            end_image = cluster_to_use[len(cluster_to_use)-1]
            begin_image_result = generate_frame_gradient(begin_image)
            end_image_result = generate_frame_gradient(end_image)
            cv2.imwrite(begin_image_name, begin_image_result)
            cv2.imwrite(end_image_name, end_image_result)
        else:
            cluster_to_use = clustered_videos[block_vid_name][block_cluster_index]
            begin_image = cluster_to_use[0]
            end_image = cluster_to_use[len(cluster_to_use)-1]
            begin_image_result = generate_frame_gradient(begin_image)
            end_image_result = generate_frame_gradient(end_image)
            cv2.imwrite(begin_image_name, begin_image_result)
            cv2.imwrite(end_image_name, end_image_result)
        counter += 1
        print "Finished the block", block_vid_name, block_cluster_index, "remain", 200-counter, "blocks to process"
    print "Spend", time.time()-start_time, "to generate cut images"


def visual_function_dispatcher(method):
    """
    Return the corresponding visual generation method
    :param method: method name string
    :return: visual generation method
    """
    if method == "gradient":
        return generate_frame_gradient
    elif method == "ellipse":
        return generate_frame_ellipse
    else:
        return generate_frame_wave


def write_frames_with_visual(result, assemble_blocks, cluster, current_progress):
    """
    Write frames from cluster to the result with DA bridges
    :param result: The result handle
    :param assemble_blocks: the abstract data result from editing
    :param current_progress: index of the current block
    """
    black_middle = np.zeros((480,640,3), np.uint8) #Frame to be inserted in the middle
    front_method = None # The DA method for the front part, its block method
    end_method = None # The DA method for the end, its next block method
    if current_progress == 0:
        end_method = assemble_blocks.editing_blocks[1].da_method
    elif current_progress == len(assemble_blocks.editing_blocks)-1:
        front_method = assemble_blocks.editing_blocks[current_progress].da_method
    else:
        front_method = assemble_blocks.editing_blocks[current_progress].da_method
        end_method = assemble_blocks.editing_blocks[current_progress+1].da_method

    num_frame = len(cluster)
    round_third = num_frame / 3

    if round_third > 24:
        print "Got Long Clip"
        front_frame_num = 24
        end_frame_num = 24
        middle_frame_num = num_frame - 48
    else:
        if round_third < 1:
            front_frame_num = 1
            middle_frame_num = 0
            end_frame_num = 0
        else:
            front_frame_num = round_third
            middle_frame_num = round_third
            end_frame_num = num_frame - front_frame_num - middle_frame_num

    front_range = range(front_frame_num)
    middle_range = range(front_frame_num, front_frame_num+middle_frame_num)
    end_range = range(middle_frame_num, end_frame_num+middle_frame_num)

    for i in range(num_frame):
        source_image = cluster[i]
        if i in front_range:
            if front_method is not None:
                print "Use front method", front_method
                visual_method_function = visual_function_dispatcher(front_method)
                result_image = visual_method_function(source_image)
                result.write(result_image)
            else:
                result.write(source_image)
        elif i in middle_range:
            result.write(black_middle)
        else:
            if end_method is not None:
                print "Use end method", end_method
                visual_method_function = visual_function_dispatcher(end_method)
                result_image = visual_method_function(source_image)
                result.write(result_image)
            else:
                result.write(source_image)
    print "Finished writing"


# Now all the middle frames are black
def generate_video_with_visuals(assemble_blocks, output):
    """
    Generate result videos with visual bridges
    :param assemble_blocks: abstract data result from editing
    :param output: output name for the video
    """
    start_time = time.time()
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    frame_rate = float(30)
    resolution = (int(640), int(480))
    visual_result = cv2.VideoWriter(output, fourcc, frame_rate, resolution)
    clustered_videos = {}
    counter = 0
    for editing_block in assemble_blocks.editing_blocks:
        print editing_block
        block_vid_name = editing_block.video_name
        block_cluster_index = editing_block.cluster_index
        block_threshold = editing_block.cluster_threshold
        if block_vid_name not in clustered_videos:
            file_name = "./test/field_test/demo_test/sub_set_1/" + block_vid_name
            camera = cv2.VideoCapture(file_name)
            cluster_start_time = time.time()
            clustered_video = cluster_with_threshold(camera, block_threshold)
            print "Spend", time.time()-cluster_start_time, "to cluster with threshold", block_threshold
            clustered_videos[block_vid_name] = clustered_video
            cluster_to_use = clustered_video[block_cluster_index]
            write_frames_with_visual(visual_result, assemble_blocks, cluster_to_use, counter)
        else:
            cluster_to_use = clustered_videos[block_vid_name][block_cluster_index]
            write_frames_with_visual(visual_result, assemble_blocks, cluster_to_use, counter)
        counter += 1
        print "Finished the block"
    print "Spend", time.time()-start_time, "to generate video"


def generate_video(assemble_blocks, output):
    """
    Generate result video and progress video from assemble_blocks class
    :param assemble_blocks: abstract data result from editing
    :param output: output name for the video
    """
    start_time = time.time()
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    frame_rate = float(30)
    resolution = (int(640), int(480))
    edited_result = cv2.VideoWriter(output, fourcc, frame_rate, resolution)
    # visual_bridge_result = cv2.VideoWriter(visual_bridge_name, fourcc, frame_rate, resolution)
    clustered_videos = {}
    for editing_block in assemble_blocks.editing_blocks:
        print editing_block
        block_vid_name = editing_block.video_name
        block_cluster_index = editing_block.cluster_index
        block_threshold = editing_block.cluster_threshold
        if block_vid_name not in clustered_videos:
            file_name = "./test/field_test/demo_test/sub_set_1/" + block_vid_name
            camera = cv2.VideoCapture(file_name)
            cluster_start_time = time.time()
            clustered_video = cluster_with_threshold(camera, block_threshold)
            # clustered_video = adaptive_cluster(camera, editing_block.cluster_threshold)
            print "Spend", time.time()-cluster_start_time, "to cluster with threshold", block_threshold
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


#------------------Test code-------------------
def test_da_methods_with_image():
    src_img = cv2.imread("./exp.jpg")
    gradient_result = generate_frame_gradient(src_img)
    ellipse_result = generate_frame_ellipse(src_img)
    wave_result = generate_frame_wave(src_img)
    cv2.imwrite("gradient_test.jpg", gradient_result)
    cv2.imwrite("ellipse_test.jpg", ellipse_result)
    cv2.imwrite("wave_test.jpg", wave_result)
    print "Finished writing test images"


def test_with_black():
    black_middle = np.zeros((480,640,3), np.uint8) #Frame to be inserted in the middle
    cv2.imshow("black", black_middle)
    cv2.waitKey(0)


# test_da_methods_with_image()
# test_with_black()


# with open("assembled_video_sub_set_1.pickle", 'rb') as input_source:
#     assembled_blocks = pickle.load(input_source)
#     generate_video_with_visuals(assembled_blocks, "subset_1_visual_with_black.mp4")

    # generate_cut_images(assembled_blocks)
    # generate_video(assembled_blocks, "subset_1_field.mp4")
    # create_cluster_sound(assembled_blocks)
    # print assembled_blocks.editing_blocks[0].video_name
    # print assembled_blocks.editing_blocks[0].cluster_index










