
"""
Code for File Watcher Module in editing system
"""

#TODO: Implement loggin for this
from kombu import Connection, Exchange, Queue
from kombu.mixins import ConsumerMixin 
import time
import message_publisher 
import sys
import os
import sqlite3
import json


def is_video(file_name):
    return file_name.endswith("avi") or file_name.endswith("mp4")


class File_Wathcher(ConsumerMixin):

    def __init__(self, connection, queues, callbacks=None):
        """
        keyword arguments:
        queues -- a list of queues the consumer will subscribe
        init -- make sure we don't reapeated sending out messages
        """
        self.connection = connection 
        self.queues = queues
        self.init = True
        if not callbacks:
            self.callbacks = [self.process_message, ]

    def get_consumers(self, Consumer, channel):
        return [Consumer(queues=self.queues, callbacks=self.callbacks)]

    def publish_message(self, routing_key, message):
        exchange = Exchange('editing_exchange', type='direct')
        file_ready_publisher = message_publisher.Message_publisher(self.connection, exchange, 
                routing_key) 
        file_ready_publisher.publishMessage(message)

    def upload_metadata(self, metadatas):
        print "start to upload the data"
        connection = sqlite3.connect('Editing.db')
        cursor = connection.cursor()
        cursor.executemany('INSERT INTO Metadata VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)', metadatas)
        connection.commit()
        connection.close()


    #TODO: Adding error handling in file checkings
    def get_metadatas(self, new_vid_names):
        '''
        :param new_vid_names:
        :return: the newly created metadata to be inserted into database
            One important thing to notice is that in this moment the to_edit range and metadata_score have not been
            calculated
            I also made the assumption that JSON data is always accompanied by video data
        '''
        metadatas = []
        for vid_name in new_vid_names:
            json_file_name = vid_name[:-4] + "_end.json"
            json_file_path = "./json/" + json_file_name
            print json_file_path
            with open(json_file_path, 'r') as file:
                vid_metadata_dict = json.load(file)
                # Now we are only using the end part
                # vid_metadata_dict = vid_metadata_dict['end']
                to_edit = None
                metadata_score = 0
                file_name = vid_name
                file_path = os.getcwd() + "/videos/" + file_name
                # Need to cast RAM_in_use_sys to int number
                ram_in_use_sys = int(vid_metadata_dict['RAM_in_use_sys'])
                vid_metadata = [file_name, file_path, vid_metadata_dict['avg_brightness'],
                                vid_metadata_dict['avg_distance'], vid_metadata_dict['motion_detected'],
                                vid_metadata_dict['entropy'], vid_metadata_dict['has_obstacle'],
                                vid_metadata_dict['position_X'], vid_metadata_dict['position_Y'],
                                vid_metadata_dict['timestamp'], vid_metadata_dict['direction'],
                                vid_metadata_dict['battery_level'], vid_metadata_dict['distance_to_dock'],
                                ram_in_use_sys, vid_metadata_dict['capture_pan_type'],
                                vid_metadata_dict['capture_duration'], metadata_score, to_edit]
                metadatas.append(vid_metadata)
                file.close()
        return metadatas

    # TODO: Adding another branch for quiting
    # TODO: Checking the pair integrity between video files and json files
    def process_message(self, body, message): 
        print "------------------ start collecting files ------------------"
        message.ack()
        # TODO: Do a sanity check for file type
        files_list = filter(is_video, os.listdir("./videos"))
        enough_quantity = body["enough_quantity"]
        print "Enough Quantity is", enough_quantity
        while True:
            current_file_list = filter(is_video, os.listdir("./videos"))
            new_added_files = list(set(current_file_list) - set(files_list))
            files_list = current_file_list
            if len(new_added_files) > 0:
                time.sleep(1) # To make sure that all the json files are transmitted
                print new_added_files
                metadatas = self.get_metadatas(new_added_files)
                self.upload_metadata(metadatas)
                print "Finished upload data"
                message = dict(ready_files=new_added_files)
                self.publish_message("data-file-ready", message)
                time.sleep(0.5)
            elif len(current_file_list) == enough_quantity and self.init:
                print "get enough data"
                message = dict(initial_file_source=current_file_list)
                self.publish_message("data-start-enough", message)
                self.init = False
                time.sleep(0.5)
            else: 
                time.sleep(1)


def main():
    ex = Exchange('editing_exchange', type='direct')
    queue_start_initialize = Queue('', exchange=ex, routing_key='data-start-initialize')
    queues = [queue_start_initialize, ]
    # Let's first work on default address
    with Connection('amqp://guest:guest@localhost:5672//') as connection:
        try: 
            file_watcher = File_Wathcher(connection, queues)
            print "start the File_Wathcher"
            file_watcher.run()
        except KeyboardInterrupt:
            sys.exit(0)


if __name__ == '__main__':
    main()




            
        
        
        
        

        







