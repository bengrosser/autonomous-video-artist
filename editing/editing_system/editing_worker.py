"""
Code for Editing worker in the editing system
"""

import editing
import sqlite3
import sys
import sets
from kombu import Connection, Exchange, Queue
from kombu.mixins import ConsumerMixin
import message_publisher


# TODO: May Need to convert the format so that all of them is list
class EditingWorker(ConsumerMixin):

    def __init__(self, connection, queues, callbacks=None):
        self.connection = connection
        self.queues = queues
        self.top_meta_file_names = []  # Used to keep track of changes in Metadata score, a list of file names, default length 10
        self.num_videos_to_use = 5  # Use this number of videos to pass into the editing function
        self.enough_data = False  # Used for initialization
        self.ff_memory = {}  # EditingWorker will keep track of the editing progress by saving ff_memory
        self.num_blocks = 100  # Used for the editing blocks to assemble blocks together
        if not callbacks:
            self.callbacks = [self.process_message, ]

    @staticmethod
    def get_top_names(top_num):
        command = 'SELECT file_name FROM Metadata ORDER BY metadata_score DESC LIMIT ' + str(top_num)
        connection = sqlite3.connect('Editing.db')
        cursor = connection.cursor()
        cursor.execute(command)
        current_top = cursor.fetchall()
        return current_top

    def top_updated(self, top_num):
        """
        This function will check whether the top ten Metascore has changed or not
        :param top_num: how many numbers of video metadata to check
        :return: whether metadata is updated
        """
        current_top = self.get_top_names(top_num)
        current_top = set(current_top)
        editing_top = set(self.top_meta_file_names)
        if current_top == editing_top:
            return False
        else:
            return True

    def merge_memory(self, new_memory):
        """
        Combine the self.ff_memory with the new_memory
        :param new_memory: The new memory generated from edit videos
        """
        total_memory = self.ff_memory.copy()
        total_memory.update(new_memory)
        self.ff_memory = total_memory

    @staticmethod
    def top_non_zero(top_num):
        """
        Check and make sure that all the top ten scores are non-zero
        :param top_num: how many numbers of video metadata to check
        :return: whether there is zero in top ten Metascore
        """
        command = 'SELECT metadata_score FROM Metadata ORDER BY metadata_score DESC LIMIT ' + str(top_num)
        connection = sqlite3.connect('Editing.db')
        cursor = connection.cursor()
        cursor.execute(command)
        current_top = cursor.fetchall()
        for metadata_score in current_top:
            if metadata_score == 0:
                return False
        return True

    def public_message(self, routing_key, message):
        exchange = Exchange('editing_exchange', type='direct')
        video_generated_publisher = message_publisher.Message_publisher(self.connection, exchange,
                                                                        routing_key)
        video_generated_publisher.publishMessage(message)

    @staticmethod
    def upload_memory(new_memory, vid_name):
        """
        Upload the memory to database
        :param new_memory: Newly generated memory
        :param vid_name: unique vid_name of time stamp plus longitude and latitude
        """
        memory_stamp = vid_name[:-4]
        memory_record = [memory_stamp, vid_name, new_memory]
        connection = sqlite3.connect('Editing.db')
        cursor = connection.cursor()
        cursor.executemany('INSERT INTO FF_Memory VALUES (?,?,?)', memory_record)
        connection.commit()
        connection.close()

    # TODO: Check and make sure that new_memory is not changed after merge
    def process_message(self, body, message):
        print "------------------ start editing process ------------------"
        # Going into the loop, every time there is Metascore update, start to produce video
        while True:
            new_memory = {}
            if not self.enough_data:
                # if data is not initialized this is the first time using it
                # self.ff_memory is the accumulated memory
                if self.top_non_zero(self.num_videos_to_use):
                    self.top_meta_file_names = self.get_top_names(self.num_videos_to_use)
                    self.enough_data = True
                    print "------------------- start editing videos ------------------"
                    new_memory, vid_name = editing.edit_videos(self.num_videos_to_use, new_memory,
                                                                   self.num_blocks, self.ff_memory)
                    self.merge_memory(new_memory)
                    self.upload_memory(new_memory, vid_name)
                    print "--------------Finished Upload Memory to Database Related To", vid_name, "------------------"
                    print "--------------Finished Producing Video With Name", vid_name, "------------"
            else:
                if self.top_updated(self.num_videos_to_use):
                    if self.top_non_zero(self.num_videos_to_use):
                        self.top_meta_file_names = self.get_top_names(self.num_videos_to_use)
                        # Now we have an update of the metadata files
                        # Since now the self.ff_memory has more than just updated memories, but also before
                        print "------------------- start editing videos ------------------"
                        new_memory, vid_name = editing.edit_videos(self.num_videos_to_use, new_memory,
                                                                       self.num_blocks, self.ff_memory)
                        self.merge_memory(new_memory)
                        self.upload_memory(new_memory, vid_name)
                        print "--------------Finished Upload Memory to Database Related To", vid_name, "-------------"
                        print "--------------Finished Producing Video With Name", vid_name, "------------"

    def get_consumers(self, Consumer, channel):
        return [Consumer(queues=self.queues, callbacks=self.callbacks)]


def main():
    ex = Exchange('editing_exchange', type='direct')
    queue_start_initialize = Queue('', exchange=ex, routing_key='data-start-enough')
    queues = [queue_start_initialize, ]
    # Let's first work on default address
    with Connection('amqp://guest:guest@localhost:5672//') as connection:
        try:
            file_watcher = EditingWorker(connection, queues)
            print "start the Editing Worker"
            file_watcher.run()
        except KeyboardInterrupt:
            sys.exit(0)
