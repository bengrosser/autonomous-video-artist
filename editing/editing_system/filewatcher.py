#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : filewatcher.py
 #Creation Date : 15-09-2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.

"""
Code for File Watcher Module in editing system
"""

#TODO: Implement Loggin for this
from kombu import Connection, Exchange, Queue
from kombu.mixins import ConsumerMixin 
import numpy as np
import time
import message_publisher 
import sys
import os
from os import path


def is_video(file_name):
    return file_name.endswith("mp4")


class File_Wathcher(ConsumerMixin): 
    def __init__(self, connection, queues, callbacks=None):
        """
        keyword arguments:
        queues -- a list of queues the consumer will subscribe
        """
        self.connection = connection 
        self.queues = queues 
        if not callbacks:
            self.callbacks = [self.process_message,]


    def get_consumers(self, Consumer, channel):
        return [Consumer(queues=self.queues, callbacks=self.callbacks)]

    
    def publish_message(self, routing_key, message):
        exchange = Exchange('editing_exchange', type='direct')
        file_ready_publisher = message_publisher.Message_publisher(self.connection, exchange, 
                routing_key) 
        file_ready_publisher.publishMessage(message)
        

    #TODO: Add functions to interact with databse
    #TODO: Adding another branch for quiting
    def process_message(self, body, message): 
        print "------------------ start collecting files ------------------"
        message.ack()
        # TODO: Do a sanity check for file type
        files_list = filter(is_video,os.listdir("./source_vid")) 
        enough_quantity = body["enough_quantity"]
        while(True):
            current_file_list = filter(is_video, os.listdir("./source_vid"))
            new_added_files = list(set(current_file_list) - set(files_list))
            files_list = current_file_list
            if len(new_added_files) > 0:
                print "new incoming files"
                message = dict(ready_files=new_added_files)
                self.publish_message("data-file-ready", message)
                time.sleep(0.5)
            elif len(current_file_list) == enough_quantity:
                print "get enough data"
                #TODO: Add function to have slicing functionality()
                #TODO: Not starting from index 0
                message = dict(initial_file_source=current_file_list)
                self.publish_message("data-start-enough", message)
                time.sleep(0.5)
            else: 
                time.sleep(1)
                pass


def main():
    ex = Exchange('editing_exchange', type='direct')
    queue_start_initialize = Queue('', exchange=ex, routing_key='data-start-initialize')
    queues = [queue_start_initialize, ]
    #Let's first work on default address
    with Connection('amqp://guest:guest@localhost:5672//') as connection:
        try: 
            file_watcher = File_Wathcher(connection, queues)
            print "start the File_Wathcher"
            file_watcher.run()
        except KeyboardInterrupt:
            sys.exit(0)


if __name__=='__main__':
    main()




            
        
        
        
        

        







