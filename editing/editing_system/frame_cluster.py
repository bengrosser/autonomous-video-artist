import cv2
import numpy as np
import random
from range_finder import calcEntropy

# TODO: Cluster video frames should take editing pair as an argument to cluster that range
# Methods from StackOverflow
def get_image_difference(image_1, image_2):
    first_image_hist = cv2.calcHist([image_1], [0], None, [256], [0, 256])
    second_image_hist = cv2.calcHist([image_2], [0], None, [256], [0, 256])
    img_hist_diff = cv2.compareHist(first_image_hist, second_image_hist, cv2.HISTCMP_BHATTACHARYYA)
    img_template_probability_match = cv2.matchTemplate(first_image_hist, second_image_hist, cv2.TM_CCOEFF_NORMED)[0][0]
    img_template_diff = 1 - img_template_probability_match
    # taking only 10% of histogram diff, since it's less accurate than template method
    commutative_image_diff = (img_hist_diff/10) + img_template_diff
    return commutative_image_diff


# randomly select one frame from each of the cluster in clustered frames
def sample_clustered_frames(frames_clusters):
    sample_result = []
    for cluster in frames_clusters:
        num_frames = len(cluster)
        if num_frames == 1:
            sample_result.append(cluster[0])
        else:
            random_index = random.randint(0, num_frames-1)
            sample_result.append(cluster[random_index])
    return sample_result


def cluster_with_threshold(camera, threshold):
    result_frames_clusters = []
    cluster_frames = []
    prev_frame = None
    while True:
        grabbed, frame = camera.read()
        if grabbed:
            # If we only have no frame to compare, we pass it into cluster frames
            if len(cluster_frames) == 0:
                cluster_frames.append(frame)
                prev_frame = frame
                continue
            else:
                image_diff = get_image_difference(frame, prev_frame)
                # print image_diff
                # Cluster breaks occurred
                if image_diff > threshold:
                    # print len(cluster_frames)
                    result_frames_clusters.append(cluster_frames)
                    cluster_frames = []
                    prev_frame = frame
                    cluster_frames.append(frame)
                else:
                    cluster_frames.append(frame)
                    prev_frame = frame
        else:
            if len(cluster_frames) > 0:
                result_frames_clusters.append(cluster_frames)
            break

    camera.set(cv2.CAP_PROP_POS_FRAMES, 0)
    return result_frames_clusters

# Computationally more intensive methods to cluster frames
def adaptive_cluster(camera, threshold):
    """
    :param file_path: File path to the video
    :param threshold: The threshold we are going to use to see if two images are similiar
    :return: clustered frames
    """
    # camera = cv2.VideoCapture(file_path)
    # result_frames_clusters = []
    # cluster_frames = []
    # prev_frame = None
    if camera is None:
        print "It can't be none here"
        return

    while True:
        result_frames_clusters = cluster_with_threshold(camera, threshold)
        camera.set(cv2.CAP_PROP_POS_FRAMES, 0)
        result_len = len(result_frames_clusters)
        # The current threshold is between 6 to 12
        if result_len > 12:
            print "Add threshold value", threshold
            print "Have", len(result_frames_clusters), "clusters"
            threshold += 0.05
            # result_frames_clusters = []
            # cluster_frames = []
        elif result_len < 6:
            print "Reduce threshold value", threshold
            print "Have", len(result_frames_clusters), "clusters"
            if threshold > 0.03:
                threshold -= 0.01
            else:
                threshold -= 0.001
            # result_frames_clusters = []
            # cluster_frames = []
        else:
            print "use threshold value", threshold
            print "Have", len(result_frames_clusters), "clusters"
            return result_frames_clusters, threshold


def cluster_with_threshold_range(camera, threshold, editing_pair):
    """
    Cluster the video clips specified in editing pair with threshold
    :param editing_pair: editing pair passed in from databse
    :return: clustered frames
    """
    result_frames_clusters = []
    cluster_frames = []
    prev_frame = None
    start_frame_num, end_frame_num = editing_pair
    num_frames_needed = end_frame_num - start_frame_num + 1
    camera.set(cv2.CAP_PROP_POS_FRAMES, start_frame_num) # Set the camera to the beginning frame
    frames_read = 0
    while True:
        if frames_read < num_frames_needed:
            grabbed, frame = camera.read()
            if grabbed:
                # If we only have no frame to compare, we pass it into cluster frames
                if len(cluster_frames) == 0:
                    cluster_frames.append(frame)
                    prev_frame = frame
                    continue
                else:
                    image_diff = get_image_difference(frame, prev_frame)
                    # print image_diff
                    # Cluster breaks occurred
                    if image_diff > threshold:
                        # print len(cluster_frames)
                        result_frames_clusters.append(cluster_frames)
                        cluster_frames = []
                        prev_frame = frame
                        cluster_frames.append(frame)
                    else:
                        cluster_frames.append(frame)
                        prev_frame = frame
                frames_read += 1
            else:
                if len(cluster_frames) > 0:
                    result_frames_clusters.append(cluster_frames)
                break
        else:
            if len(cluster_frames) > 0:
                result_frames_clusters.append(cluster_frames)
            break
    camera.set(cv2.CAP_PROP_POS_FRAMES, 0)
    return result_frames_clusters


# Current threshold range for integrated test is set at (3, 8)
def adaptive_cluster_with_range(camera, threshold, editing_pair, length_pair):
    """
    :param file_path: File path to the video
    :param threshold: The threshold we are going to use to see if two images are similiar
    :param length_pair: The ideal length range of the result cluster
    :return: clustered frames
    """
    # camera = cv2.VideoCapture(file_path)
    # result_frames_clusters = []
    # cluster_frames = []
    # prev_frame = None

    print "Clustering with range", editing_pair
    print "Length in the range", length_pair
    if camera is None:
        print "It can't be none here"
        return
    loop_security_valve = 50 # Make sure the clustering algorithm doesn't stuck in the infinite loop
    loop_count = 0
    while True:
        result_frames_clusters = cluster_with_threshold_range(camera, threshold, editing_pair)
        camera.set(cv2.CAP_PROP_POS_FRAMES, 0)
        result_len = len(result_frames_clusters)
        if loop_count == loop_security_valve:
            print "use threshold value", threshold
            print "Have", len(result_frames_clusters), "clusters"
            return result_frames_clusters, threshold
        # The current threshold is between 3 to 8
        # For some of the single frame block, cluster numbers will not change at all
        if threshold <= 0:
            print "use threshold value", threshold
            print "Have", len(result_frames_clusters), "clusters"
            return result_frames_clusters, threshold
        else:
            if result_len > length_pair[1]:
                print "Add threshold value", threshold
                print "Have", len(result_frames_clusters), "clusters"
                threshold += 0.05
                # result_frames_clusters = []
                # cluster_frames = []
            elif result_len < length_pair[0]:
                print "Reduce threshold value", threshold
                print "Have", len(result_frames_clusters), "clusters"
                if threshold > 0.03:
                    threshold -= 0.01
                else:
                    threshold -= 0.001
                # result_frames_clusters = []
                # cluster_frames = []
            else:
                print "use threshold value", threshold
                print "Have", len(result_frames_clusters), "clusters"
                return result_frames_clusters, threshold
        loop_count += 1


def cluster_video_frames_entropy(camera, n):
    """
    :param file_path: Path of the video to cluster
    :param n: tolerance of std, how many tolerance we think it belongs to another cluster
    :return: clustered frames
    """
    # camera = cv2.VideoCapture(file_path)
    result_frames_clusters = []
    result_entropy_clusters = []
    cluster_entropy = []
    cluster_frames = []
    while True:
        grabbed, frame = camera.read()
        if grabbed:
            frame_entropy = calcEntropy(frame)
            if len(cluster_entropy) <= 30:
                cluster_entropy.append(frame_entropy)
                cluster_frames.append(frame)
                continue
            mean = np.mean(np.array(cluster_entropy))
            std = np.std(np.array(cluster_entropy))
            if abs(mean - frame_entropy) > n * std:    # Check the tolerance
                result_frames_clusters.append(cluster_frames)
                result_entropy_clusters.append(cluster_entropy)
                cluster_frames = []
                cluster_entropy = []
            cluster_entropy.append(frame_entropy)
            cluster_frames.append(frame)
        else:
            result_frames_clusters.append(cluster_frames)
            result_entropy_clusters.append(cluster_entropy)
            print "Finished reading"
            break

    return result_frames_clusters


def get_block_time_range(clusters, cluster_index, fps):
    """
    Get the time range of a block from clusters based upon fps
    :param clusters: clustered video
    :param cluster_index: which index this block belongs to
    :param fps: FPS of the video
    :return: the time range for pydub to use
    """
    total_frames_before = 0
    for i in range(cluster_index):
        total_frames_before += len(clusters[i])
    begin_time = (total_frames_before/fps)*1000
    end_time = ((total_frames_before + len(clusters[cluster_index]))/fps)*1000
    return begin_time, end_time



# camera = cv2.VideoCapture("./test/field_test/demo_test/lobby1/clip6.avi")
# clustered_result, threshold = adaptive_cluster(camera, 0.04)
# while True:
#     grabbed, frame = camera.read()
#     if grabbed:
#         print "Got a frame"
#     else:
#         print "No frame for old man"
#         break
#
# camera.set(cv2.CAP_PROP_POS_FRAMES, 0)
# print "Now second time"
# while True:
#     grabbed, frame = camera.read()
#     if grabbed:
#         print "Got a frame second"
#     else:
#         print "No frame for super old man"
#         break

# camera = cv2.VideoCapture("./test/field_test/demo_test/sub_set_2/clip6.avi")
# clustered_result, threshold  = adaptive_cluster(camera, 0.05)
# print threshold
# print len(result)
# counter = 0
# for result in clustered_result:
#     for frame in result:
#         window_name = "Cluster " + str(counter)
#         cv2.imshow(window_name, frame)
#         cv2.waitKey(1)
#     counter += 1
# cv2.destroyAllWindows()
