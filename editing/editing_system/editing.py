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
#   Build the first two EditingBlocks based upon Initialization stage output
#   Put all things together with AssembledBlocks
import cv2
import epf
import time
import json
import pickle
import frame_cluster
from editing_structures import EditingBlock, AssembledBlocks
from epf import vector_magnitude


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
            print "One pair spends" + time.time() - start_time, "seconds"
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
    lowest_vid1_threshold = None
    lowest_vid2_threshold = None
    for i in range(num_of_videos):
        for j in range(i + 1, num_of_videos):
            print "Start to work on", vid_names[i], vid_names[j]
            start_time = time.time()
            vid1_threshold, vid2_threshold, local_lowest_pair_key_1, local_lowest_pair_key_2, local_vid1_cluster_offset, local_vid2_cluster_offset, \
            local_lowest_vector, local_lowest_magnitude, ff_memory_to_update = \
                epf.gradient_epf_with_generators(vid_names[i], imported_video_sources[vid_names[i]], vid_names[j],
                                                 imported_video_sources[vid_names[j]], ff_memory_to_update)
            print "Spend", time.time() - start_time, "seconds on pair", vid_names[i], vid_names[j]
            if local_lowest_magnitude < lowest_magnitude:
                lowest_vector_vid1_key = vid_names[i]
                lowest_vector_vid2_key = vid_names[j]
                lowest_vector_vid1_pair_key = local_lowest_pair_key_1
                lowest_vector_vid2_pair_key = local_lowest_pair_key_2
                lowest_vector_vid1_offset = local_vid1_cluster_offset
                lowest_vector_vid2_offset = local_vid2_cluster_offset
                lowest_vector = local_lowest_vector
                lowest_magnitude = local_lowest_magnitude
                lowest_vid1_threshold = vid1_threshold
                lowest_vid2_threshold = vid2_threshold
    return lowest_vid1_threshold, lowest_vid2_threshold, lowest_vector_vid1_key, lowest_vector_vid2_key, lowest_vector_vid1_pair_key, lowest_vector_vid2_pair_key, \
           lowest_vector_vid1_offset, lowest_vector_vid2_offset, lowest_vector, ff_memory_to_update


# TODO: Add the third method
def find_best_editing_pair(ff_memory):
    """
    Go through three categories of ff_memory data and find the best data by comparing
    smallest / average  score
    :param ff_memory: the ff_memory generated by epf with generators
    :return: the key and the category of the methods
    """
    gradient_key, min_gradient_value = min(ff_memory.items(), key=lambda k: vector_magnitude(k[1]['gradient']))
    ellipse_key, min_ellipse_value = min(ff_memory.items(), key=lambda k: k[1]['ellipse'])
    wave_key, min_wave_value = min(ff_memory.items(), key=lambda k: k[1]['wave'])

    min_gradient_value = vector_magnitude(min_gradient_value['gradient'])
    min_ellipse_value = min_ellipse_value['ellipse']
    min_wave_value = min_wave_value['wave']



    num_items = len(ff_memory.keys())
    total_gradient_sum = 0
    total_ellipse_sum = 0
    total_wave_sum = 0
    for key in ff_memory.keys():
        total_gradient_sum += vector_magnitude(ff_memory[key]['gradient'])
        total_ellipse_sum += ff_memory[key]['ellipse']
        total_wave_sum += ff_memory[key]['wave']

    gradient_average = total_gradient_sum / float(num_items)
    ellipse_average = total_ellipse_sum / float(num_items)
    wave_average = total_wave_sum / float(num_items)

    # Take notice here, here the smaller the better
    gradient_score = min_gradient_value / gradient_average
    ellipse_score = min_ellipse_value / ellipse_average
    wave_score = min_wave_value / wave_average

    print gradient_score
    print ellipse_score
    print wave_score

    if gradient_score < ellipse_score and gradient_score < wave_score:
        return gradient_key, "gradient"
    elif ellipse_score < gradient_score and ellipse_score < wave_score:
        return ellipse_key, "ellipse"
    else:
        return wave_key, "wave"


# Use this function in the editing process
def editing_generator_initialize(imported_video_sources, ff_memory_to_update):
    """
    :param imported_video_sources: the dictionary with format {vid_name: [[editing pairs], video generator]]}
    :param ff_memory_to_update: used to book keeping comparison results
    :return: coordinates for the most, ff_memory
    """
    vid_names = imported_video_sources.keys()
    num_of_videos = len(vid_names)
    process_start_time = time.time()
    counter = 1
    total_pair = 30
    for i in range(num_of_videos):
        for j in range(i + 1, num_of_videos):
            print "Start to work on", vid_names[i], vid_names[j]
            start_time = time.time()
            print "Spend", time.time() - process_start_time, "seconds on the process------"
            # if imported_video_sources[vid_names[i]][1] is None or imported_video_sources[vid_names[j]][1] is None:
            #     imported_videos_sources = test_import_generator("./test/field_test/demo_test/sub_set_1")
            #     print "Re import!!!!!!!!!!!!!!!!!!!!"
            ff_memory_to_update = epf.epf_with_generators(vid_names[i], imported_video_sources[vid_names[i]], vid_names[j],
                                                 imported_video_sources[vid_names[j]], ff_memory_to_update)
            print "Spend", time.time() - start_time, "seconds on pair", vid_names[i], vid_names[j]
            print "-------------------------------------------------"
            print "-------------------------------------------------"
            print "-------------------------------------------------"
            print "Progress", 100*float(counter)/total_pair, "%"
            print "-------------------------------------------------"
            print "-------------------------------------------------"
            print "-------------------------------------------------"
            counter += 1

    print "Finished Producing ff_memory"
    with open("ff_memory_sub_set_1.pickle", 'wb') as descriptor:
        pickle.dump(ff_memory_to_update, descriptor, pickle.HIGHEST_PROTOCOL)
        print "Finished writing ff_memory"
    memory_key, method = find_best_editing_pair(ff_memory_to_update)
    return memory_key, method, ff_memory_to_update


# Editing Stage
def edit_videos():
    start_time = time.time()
    imported_videos_sources = test_import_generator("./test/field_test/demo_test/sub_set_1")
    # imported_videos_sources = test_import("./test")
    print "Spend", time.time() - start_time, "seconds to import videos"
    print "Start videos quality check"
    check_start_time = time.time()
    success, which_key = video_quality_check(imported_videos_sources)
    if not success:
        print which_key, "can't be read multiple times, choose another one"
        return
    else:
        print "All Videos are good to go"
    print "Spend", time.time() - check_start_time, "seconds to test videos"
    ff_memory = {}
    used_keys = []
    # Previous method
    # lowest_vid1_threshold, lowest_vid2_threshold, result_vid1_key, result_vid2_key, result_vid1_pair_key, result_vid2_pair_key, result_vid1_cluster_offset, \
    # result_vid2_cluster_offset, result_vector, ff_memory  = generator_initialize(imported_videos_sources, ff_memory)
    # print "Spend", time.time() - start_time, "seconds to finish first two stages of editing"
    # block_1 = EditingBlock(result_vid1_key, result_vid1_pair_key, 'gradient', result_vid1_cluster_offset, lowest_vid1_threshold)
    # block_2 = EditingBlock(result_vid2_key, result_vid2_pair_key, 'gradient', result_vid2_cluster_offset, lowest_vid2_threshold)
    # first_key = (result_vid1_key, result_vid2_key, result_vid1_pair_key, result_vid2_pair_key,
    #              result_vid1_cluster_offset, result_vid2_cluster_offset, lowest_vid1_threshold, lowest_vid2_threshold)

    initialization_start_time = time.time()
    first_key, method, ff_memory = editing_generator_initialize(imported_videos_sources, ff_memory)
    print "Spend", time.time() - initialization_start_time, "seconds to initialize editing process"
    block_1 = EditingBlock(first_key[0], first_key[2], method, first_key[4], first_key[6])
    block_2 = EditingBlock(first_key[1], first_key[3], method, first_key[5], first_key[7])
    used_keys.append(first_key)
    assembled_blocks = AssembledBlocks(block_1, block_2, ff_memory, used_keys)
    assembled_blocks.assemble_blocks(100)
    print "Spend", time.time() - start_time, "seconds to to finish editing"
    with open("assembled_video_sub_set_1.pickle", 'wb') as descriptor:
        pickle.dump(assembled_blocks, descriptor, pickle.HIGHEST_PROTOCOL)


# ---------------------TEST CODES-----------------------
def test_assemble():
    imported_videos_sources = test_import_generator("./test/field_test/demo_test/sub_set_1")
    used_keys = []
    with open("ff_memory_sub_set_1.pickle", 'rb') as input:
        ff_memory = pickle.load(input)
    first_key, method = find_best_editing_pair(ff_memory)
    block_1 = EditingBlock(first_key[0], first_key[2], method, first_key[4], first_key[6])
    block_2 = EditingBlock(first_key[1], first_key[3], method, first_key[5], first_key[7])
    used_keys.append(first_key)
    assembled_blocks = AssembledBlocks(block_1, block_2, ff_memory, used_keys)
    assembled_blocks.assemble_blocks(100)
    with open("assembled_video_sub_set_1.pickle", 'wb') as descriptor:
        pickle.dump(assembled_blocks, descriptor, pickle.HIGHEST_PROTOCOL)


def video_read_through(video_generator):
    """
    Read through the video and reset the generator
    :param video_generator: the generator to read from
    :return: success read through?  and frames length to compare multiple times
    """
    total_frames = 0
    while True:
        grabbed, frame = video_generator.read()
        if grabbed:
            total_frames += 1
        else:
            print "Read Through in progress"
            break
    video_generator.set(cv2.CAP_PROP_POS_FRAMES, 0)
    if total_frames > 0:
        return True, total_frames
    else:
        return False, total_frames


def video_quality_check(imported_videos_sources):
    """
    Check and make sure the quality of the video is the same through multiple read
    :param imported_video_sources: the dictionary with format {vid_name: [[editing pairs], video generator]]}
    :return: whether all videos have good qualities to be read multiple times, if not which one is fucked up
    """
    vid_keys = imported_videos_sources.keys()
    for key in vid_keys:
        generator = imported_videos_sources[key][1]
        success_through, frames_num = video_read_through(generator)
        if success_through:
            for i in range(3):
                success_again, next_frames_num = video_read_through(generator)
                if not success_again or next_frames_num != frames_num:
                    print key, "can't be read multiple times"
                    return False, key
    return True, None


def test_1():
    imported_videos_sources = test_import_generator("./test/field_test/demo_test/sub_set_1")
    video_1k = imported_videos_sources.keys()[0]
    video_2k = imported_videos_sources.keys()[1]
    vid1_generator = imported_videos_sources[video_1k][1]
    vid2_generator = imported_videos_sources[video_2k][1]
    vid1_generator.set(cv2.CAP_PROP_POS_FRAMES, 0)
    vid2_generator.set(cv2.CAP_PROP_POS_FRAMES, 0)
    clustered_vid1 = frame_cluster.adaptive_cluster(vid1_generator, 0.25)
    clustered_vid2 = frame_cluster.adaptive_cluster(vid2_generator, 0.25)
    sampled_frames_cluster1 = frame_cluster.sample_clustered_frames(clustered_vid1)
    sampled_frames_cluster2 = frame_cluster.sample_clustered_frames(clustered_vid2)
    vid1_generator.set(cv2.CAP_PROP_POS_FRAMES, 0)
    vid2_generator.set(cv2.CAP_PROP_POS_FRAMES, 0)


def test_2():
    start_time = time.time()
    imported_videos_sources = test_import_generator("./test/field_test/demo_test/sub_set_1")
    # imported_videos_sources = test_import("./test")
    print "Spend", time.time() - start_time, "seconds to import videos"
    ff_memory = {}
    vid_names = imported_videos_sources.keys()
    for key in vid_names:
        if imported_videos_sources[key][1] is None:
            print "Fuck yeah"
        else:
            print key



# edit_videos()
test_assemble()

# imported_videos_sources = test_import_generator("./test/field_test/demo_test/sub_set_2")
# success, which_key = video_quality_check(imported_videos_sources)
# if success:
#     print "All Videos are good to go"
# else:
#     print which_key, "has issues"


# with open("ff_memory_sub_set_1.pickle", "rb") as input_source:
#     test_ff_memory = pickle.load(input_source)
#     keys = test_ff_memory.keys()
#     for key in keys:
#         print test_ff_memory[key]['wave']




# test_import_generator("./test/field_test/demo_test/sub_set_1")

# start_time = time.time()
# imported_videos_sources = test_import_generator("./test")
# # imported_videos_sources = test_import("./test")
# print "Spend", time.time()-start_time, "seconds to import videos"
# ff_memory = {}
# result_vid1_key, result_vid2_key, result_vid1_pair_key, result_vid2_pair_key, result_vid1_offset, result_vid2_offset, \
#     result_vector, ff_memory = generator_initialize(imported_videos_sources, ff_memory, 0.025)
# print "Spend", time.time()-start_time, "seconds to finish first two stages of editing"
# print result_vid1_key, result_vid2_key, result_vid1_pair_key, result_vid2_pair_key, result_vid1_offset, \
#     result_vid2_offset, result_vector
# print "There are", len(ff_memory), "items in ff_memory"
# with open('ff_memory.pickle', 'wb') as descriptor:
#     cPickle.dump(ff_memory, descriptor, protocol=cPickle.HIGHEST_PROTOCOL)
