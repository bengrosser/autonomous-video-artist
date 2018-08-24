"""
Code for utilities that can edit movies together
"""
import editing
import epf
import cv2
from editing_structures import EditingBlock, AssembledBlocks
from produce_result import generate_video_with_range, generate_video_with_visuals_with_range


def movies_editing(movie1_name, movie2_name, movie1_path, movie2_path, frame_rate, resolution, length_pair, num_blocks,
                   src_path, output_path):
    """
    This function will edit movies in movie1_path and movie2_path together. The cluster range tells
    the clustering algorithm how many clusters we want from each movie
    :param movie1_name: name of movie1
    :param movie2_name: name of movie2
    :param movie1_path: path for movie1
    :param movie2_path: path for movie2
    :param frame_rate: frame rate of the source movies
    :param resolution: resolution of the source movies
    :param length_pair: ideal length for the clustered videos
    :param num_blocks: control the length of the new video (not precise)
    :param src_path: where to read the video (for generate video)
    :param output_path: where to put the generated video
    """
    movie1_generator = editing.load_generator(movie1_path)
    movie2_generator = editing.load_generator(movie2_path)
    movie1_length = int(movie1_generator.get(cv2.CAP_PROP_FRAME_COUNT))
    movie2_length = int(movie2_generator.get(cv2.CAP_PROP_FRAME_COUNT))
    movie1_range = (1000, 2000)
    movie2_range = (4000, 5000)
    ff_memory = {}
    total_memory = {}  # A prop to be passed into the epf.editing_compare_frame_cluster
    ff_memory = epf.editing_compare_frame_cluster(movie1_name, movie1_generator, movie1_range, movie2_name,
                                                  movie2_generator,
                                                  movie2_range, ff_memory, total_memory, length_pair)
    first_key, method = editing.find_best_editing_pair(ff_memory)
    used_keys = [first_key]
    block_1 = EditingBlock(first_key[0], first_key[2], method, first_key[4], first_key[6])
    block_2 = EditingBlock(first_key[1], first_key[3], method, first_key[5], first_key[7])
    used_keys.append(first_key)
    assembled_blocks = AssembledBlocks(block_1, block_2, ff_memory, used_keys)
    assembled_blocks.assemble_blocks(num_blocks)
    generate_video_with_range(assembled_blocks, frame_rate, resolution, src_path, output_path)
    print "Finished Editing Movies"


def main():
    video_root_path = "/Users/frankshammer42/Documents/CS/autonomous-video-artist/editing/editing_system/movie_testing/"  # The path of the directory that contains both movie files
    movie_1_path = video_root_path + "ex.mp4"
    movie_2_path = video_root_path + "her.mp4"
    length_pair = (15, 24)  # to control the clustering of the movie
    num_blocks = 60
    output_path = "./movie_test_result.mp4"
    frame_rate = 24
    resolution = (1920, 1040)
    movies_editing("ex.mp4", "her.mp4", movie_1_path, movie_2_path, frame_rate, resolution, length_pair, num_blocks,
                   video_root_path, output_path)


if __name__ == '__main__':
    main()
