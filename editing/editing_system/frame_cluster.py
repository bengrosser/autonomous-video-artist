import cv2
import numpy as np
import random
from range_finder import calcEntropy

# TODO: Change the threshold along the way if there are too much segments
# TODO: Cluster video frames should take editing pair as an argument to cluster that range
# Methods from StackOverflow
def get_image_difference(image_1, image_2):
    first_image_hist = cv2.calcHist([image_1], [0], None, [256], [0, 256])
    second_image_hist = cv2.calcHist([image_2], [0], None, [256], [0, 256])
    img_hist_diff = cv2.compareHist(first_image_hist, second_image_hist, cv2.cv.CV_COMP_BHATTACHARYYA)
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


# Computationally more intensive methods to cluster frames
def cluster_video_frames_intense(camera, threshold):
    """
    :param file_path: File path to the video
    :param threshold: The threshold we are going to use to see if two images are similiar
    :return: clustered frames
    """
    # camera = cv2.VideoCapture(file_path)
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
            result_frames_clusters.append(cluster_frames)
            break
    return result_frames_clusters


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


# frames_result = cluster_video_frames_intense('./test/2958.mp4', 0.025)
# counter = 0
# for frames_cluster in frames_result:
#     for frame in frames_cluster:











































