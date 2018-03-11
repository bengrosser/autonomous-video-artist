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
            return (key[1], key[3], key[5], key[6])
        else:
            return (key[0], key[2], key[4], key[7])

    # TODO: Now I only have gradient methods, later need to update this function to include the other two
    # Find next lowest score / best compatible block
    def find_best_compatible_block(self, ff_memory, used_keys):
        """
        :param ff_memory: frame(cluster) comparison results from the initialization stage
        :param used_keys: representation of the blocks that have already been used in assembled blocks
        :return: the block info of the key in the ff_memory that has is most compatible with the block, and its vector
            magnitude, also the key that the block generated from
        """
        block_id = {self.video_name, self.clip_range, self.cluster_index, self.cluster_threshold}
        # lowest_vector = None
        lowest_vector_magnitude = float('Inf')
        lowest_key = None
        memory_key = None
        for key in ff_memory:
            if key in used_keys:
                continue
            key_id = set(key)
            if block_id.issubset(key_id):
                similarity_vector = ff_memory[key]['gradient']
                similarity_magnitude = vector_magnitude(similarity_vector)
                if similarity_magnitude < lowest_vector_magnitude:
                    # lowest_vector = similarity_vector
                    lowest_vector_magnitude = similarity_magnitude
                    lowest_key = self.get_block_info_from_key(key)
                    memory_key = key
        return lowest_key, lowest_vector_magnitude, memory_key


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

    def __repr__(self):
        assembled_repr = ""
        for block in self.editing_blocks:
            assembled_repr += repr(block)
            assembled_repr += "\n"
        return assembled_repr

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
            for block in self.editing_blocks:
                block_lowest_key, block_lowest_magnitude, block_memory_key = \
                    block.find_best_compatible_block(self.source_memory, self.used_keys)
                lowest_keys.append(block_lowest_key)
                lowest_magnitude.append(block_lowest_magnitude)
                lowest_memory_keys.append(block_memory_key)
            lowest_index = lowest_magnitude.index(min(lowest_magnitude))
            lowest_key = lowest_keys[lowest_index]
            lowest_memory_key = lowest_memory_keys[lowest_index]
            new_block = EditingBlock(lowest_key[0], lowest_key[1], 'gradient', lowest_key[2], lowest_key[3])
            print "Created a new block"
            print new_block
            print "Will be inserted at", lowest_index
            print "\n"
            self.editing_blocks.insert(lowest_index+1, new_block)
            self.used_keys.append(lowest_memory_key)
        print "Assembling Finished"
