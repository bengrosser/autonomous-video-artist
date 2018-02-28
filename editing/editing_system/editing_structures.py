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

    # TODO: Now I only have gradient methods, later need to update this function to include the other two
    def find_best_compatible_block(self, ff_memory, used_keys):
        """
        :param ff_memory: frame(cluster) comparison results from the initialization stage
        :param used_keys: representation of the blocks that have already been used in assembled blocks
        :return: the block info of the key in the ff_memory that has is most compatible with the block
        """









