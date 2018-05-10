from epf import vector_magnitude


class EditingBlock:
    """
    A Block that stores all the necessary information for
    reconstructing video
    """

    def __init__(self, video_name, editing_key, which_method, cluster_index, threshold):
        """
        :param video_name: which video this block comes from
        :param editing_key: from which sub-clip of this video
        :param which_method: which DA method is used for choosing this block
        :param cluster_index: which cluster identifies this block (to recover)
        :param threshold: which threshold is used to cluster sub-clips
        """
        self.video_name = video_name
        self.clip_range = editing_key
        self.da_method = which_method
        self.cluster_index = cluster_index
        self.cluster_threshold = threshold

    def __repr__(self):
        border = "####################################################\n"
        name_repr = "From Video " + self.video_name + "\n"
        range_repr = "From Range " + str(self.clip_range) + "\n"
        method_repr = "Extracted by " + str(self.da_method) + " DA Method" + "\n"
        cluster_repr = "From Cluster " + str(self.cluster_index) + "\n"
        threshold_repr = "Clustered by Threshold of " + str(self.cluster_threshold)
        result = border + name_repr + range_repr + method_repr + cluster_repr + threshold_repr
        return result

    # Based on self's info, extract another block's information from the key
    def get_block_info_from_key(self, key):
        """
        :param key: A key from ff_memory that contains self's info
        :return: a tuple with another block's info
        """
        if key[0] == self.video_name:
            return key[1], key[3], key[5], key[7]
        else:
            return key[0], key[2], key[4], key[6]

    # Find next lowest score / best compatible block
    def find_best_compatible_block(self, ff_memory, used_keys, gradient_avg, ellipse_avg, wave_avg):
        """
        :param ff_memory: frame(cluster) comparison results from the initialization stage
        :param used_keys: representation of the blocks that have already been used in assembled blocks
        :param gradient_avg: average value from ff_memory of gradient score
        :param ellipse_avg: average value from ff_memory of ellipse score
        :param wave_avg: average value from ff_memory of wave score
        :return: the block info of the key in the ff_memory that has is most compatible with the block, the magnitude should
            be compared to average, the method that creates this value
        """

        # For Normalizing the data
        gradient_key, min_gradient_value = min(ff_memory.items(), key=lambda k: vector_magnitude(k[1]['gradient']))
        ellipse_key, min_ellipse_value = min(ff_memory.items(), key=lambda k: k[1]['ellipse'])
        wave_key, min_wave_value = min(ff_memory.items(), key=lambda k: k[1]['wave'])

        min_gradient_value = vector_magnitude(min_gradient_value['gradient'])
        min_ellipse_value = min_ellipse_value['ellipse']
        min_wave_value = min_wave_value['wave']

        gradient_key, max_gradient_value = max(ff_memory.items(), key=lambda k: vector_magnitude(k[1]['gradient']))
        ellipse_key, max_ellipse_value = max(ff_memory.items(), key=lambda k: k[1]['ellipse'])
        wave_key, max_wave_value = max(ff_memory.items(), key=lambda k: k[1]['wave'])

        max_gradient_value = vector_magnitude(max_gradient_value['gradient'])
        max_ellipse_value = max_ellipse_value['ellipse']
        max_wave_value = max_wave_value['wave']

        block_id = {self.video_name, self.clip_range, self.cluster_index, self.cluster_threshold}
        # lowest_vector = None
        gradient_lowest_vector_magnitude = float('Inf')
        gradient_lowest_key = None
        gradient_memory_key = None

        ellipse_lowest_magnitude = float('Inf')
        ellipse_lowest_key = None
        ellipse_memory_key = None

        wave_lowest_magnitude = float('Inf')
        wave_lowest_key = None
        wave_memory_key = None

        for key in ff_memory:
            if key in used_keys:
                continue
            key_id = set(key)
            if block_id.issubset(key_id):
                similarity_vector = ff_memory[key]['gradient']
                similarity_magnitude = vector_magnitude(similarity_vector)
                ellipse_magnitude = ff_memory[key]['ellipse']
                wave_magnitude = ff_memory[key]['wave']
                if similarity_magnitude < gradient_lowest_vector_magnitude:
                    # lowest_vector = similarity_vector
                    gradient_lowest_vector_magnitude = similarity_magnitude
                    gradient_lowest_key = self.get_block_info_from_key(key)
                    gradient_memory_key = key
                if ellipse_magnitude < ellipse_lowest_magnitude:
                    ellipse_lowest_magnitude = ellipse_magnitude
                    ellipse_lowest_key = self.get_block_info_from_key(key)
                    ellipse_memory_key = key
                if wave_magnitude < wave_lowest_magnitude:
                    wave_lowest_magnitude = wave_magnitude
                    wave_lowest_key = self.get_block_info_from_key(key)
                    wave_memory_key = key

        # ellipse_score = ellipse_lowest_magnitude / ellipse_avg
        # gradient_score = gradient_lowest_vector_magnitude / gradient_avg
        # wave_score = wave_lowest_magnitude / wave_avg

        ellipse_score = (ellipse_lowest_magnitude - min_ellipse_value) / (max_ellipse_value - min_ellipse_value)
        gradient_score = (gradient_lowest_vector_magnitude - min_gradient_value) / (
                    max_gradient_value - min_gradient_value)
        wave_score = (wave_lowest_magnitude - min_wave_value) / (max_wave_value - min_wave_value)

        # print ellipse_score, gradient_score, wave_score

        if ellipse_score < gradient_score and ellipse_score < wave_score:
            # print "Scaled value is", ellipse_lowest_magnitude / ellipse_avg
            # print "Using ellipse method"
            return ellipse_lowest_key, ellipse_score, ellipse_memory_key, "ellipse"
        elif gradient_score < ellipse_score and gradient_score < wave_score:
            # print "Scaled value is", gradient_lowest_vector_magnitude / gradient_avg
            # print "Using gradient method"
            return gradient_lowest_key, gradient_score, gradient_memory_key, "gradient"
        else:
            return wave_lowest_key, wave_score, wave_memory_key, "wave"


# TODO: Add another method
class AssembledBlocks:
    """
    Data structure that holds the assembled blocks that
    are going to generate the final video
    """

    def __init__(self, block_1, block_2, ff_memory, used_keys):
        """
        Uses the best pair generated from initialization phase to initialize the class
        :param block_1: EditingBlock_1
        :param block_2: EditingBlock_2
        :param ff_memory: Updated ff_memory generated from initialization stage
        :param used_keys: representation of the blocks that have already been used in assembled blocks
        """
        self.editing_blocks = []
        self.source_memory = ff_memory  # The whole assembly will be based on the ff_memory
        self.editing_blocks.append(block_1)
        self.editing_blocks.append(block_2)
        self.used_keys = used_keys
        self.avg_gradient = 0.0
        self.avg_ellipse = 0.0
        self.avg_wave = 0.0
        self.get_average()

    def __repr__(self):
        assembled_repr = ""
        for block in self.editing_blocks:
            assembled_repr += repr(block)
            assembled_repr += "\n"
        return assembled_repr

    def get_average(self):
        total_gradient_sum = 0.0
        total_ellipse_sum = 0.0
        total_wave_sum = 0.0
        num_items = len(self.source_memory)
        for key in self.source_memory.keys():
            total_gradient_sum += vector_magnitude(self.source_memory[key]['gradient'])
            total_ellipse_sum += self.source_memory[key]['ellipse']
            total_wave_sum += self.source_memory[key]['wave']

        self.avg_gradient = total_gradient_sum / float(num_items)
        self.avg_ellipse = total_ellipse_sum / float(num_items)
        self.avg_wave = total_wave_sum / float(num_items)

    def assemble_blocks(self, num_blocks):
        """
        Assemble blocks based on the initial blocks and ff_memory, the assemble should go behind, different with
            insertion
        :param num_blocks: number of blocks needs to be assembled
        """
        while len(self.editing_blocks) != num_blocks:
            lowest_keys = []  # Contain another block's info
            lowest_magnitude = []
            lowest_memory_keys = []  # Contain info about the source_memory key
            lowest_methods = []
            for block in self.editing_blocks:
                block_lowest_key, block_lowest_magnitude, block_memory_key, block_method = \
                    block.find_best_compatible_block(self.source_memory, self.used_keys, self.avg_gradient,
                                                     self.avg_ellipse, self.avg_wave)
                lowest_keys.append(block_lowest_key)
                lowest_magnitude.append(block_lowest_magnitude)
                lowest_memory_keys.append(block_memory_key)
                lowest_methods.append(block_method)
            lowest_index = lowest_magnitude.index(min(lowest_magnitude))
            lowest_key = lowest_keys[lowest_index]
            lowest_memory_key = lowest_memory_keys[lowest_index]
            lowest_method = lowest_methods[lowest_index]
            new_block = EditingBlock(lowest_key[0], lowest_key[1], lowest_method, lowest_key[2], lowest_key[3])
            print "Created a new block"
            print new_block
            print "Will be inserted at", lowest_index
            print "\n"
            self.editing_blocks.insert(lowest_index + 1, new_block)
            self.used_keys.append(lowest_memory_key)
        print "Assembling Finished"
