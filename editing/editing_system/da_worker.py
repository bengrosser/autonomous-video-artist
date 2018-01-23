"""
Code for deep analysis video worker in the editing system
"""

from kombu import Connection, Exchange, Queue
from kombu.mixins import ConsumerMixin
import os
import gradient
import ellipse
import sys
import message_publisher


class DAvideoWorker(ConsumerMixin):

    def __init__(self, connection, queues, callbacks=None):
        self.connection = connection
        self.queues = queues
        if not callbacks:
            self.callbacks = [self.process_message, ]

    @staticmethod
    def generate_gradient_videos(vids_names):
        '''
        Produce gradient analysis videos for newly added vids_names
        The framerate and resolution 1 and 2 are hardcoded for now
        '''
        res1 = 640
        res2 = 480
        framerate = 30
        for src_name in vids_names:
            src_root_dir_name = src_name[:-4]
            src_root_dir_path = "./da_video_result/" + src_root_dir_name
            if not os.path.exists(src_root_dir_path):
                os.makedirs(src_root_dir_path)
            output_path = src_root_dir_path + "/gradient_video.mp4"
            gradient.produce_gradient_video(src_name, output_path, framerate, res1, res2)
        print "Finished producing gradient videos"

    @staticmethod
    def generate_ellipse_videos(vids_names):
        '''
        Produce feature analysis videos for newly added vids_names
        The framerate and resolution 1 and 2 are hardcoded for now
        '''
        res1 = 640
        res2 = 480
        framerate = 30
        for src_name in vids_names:
            src_root_dir_name = src_name[:-4]
            src_root_dir_path = "./da_video_result/" + src_root_dir_name
            if not os.path.exists(src_root_dir_path):
                os.makedirs(src_root_dir_path)
            output_path = src_root_dir_path + "/ellipse_video.mp4"
            ellipse.produce_ellipse_video(src_name, output_path, framerate, res1, res2)
        print "Finished producing ellipse videos"

    def publish_message(self, routing_key, message):
        exchange = Exchange('editing_exchange', type='direct')
        file_ready_publisher = message_publisher.Message_publisher(self.connection, exchange,
                                                                   routing_key)
        file_ready_publisher.publishMessage(message)

    def process_message(self, body, message):
        new_added_vids_name = body["ready_files"]
        print "Got the message. Start to produce DA videos"
        print "Start producing ellipse videos"
        self.generate_ellipse_videos(new_added_vids_name)
        finished_ellipse_message = dict(finished_ellipse_files=new_added_vids_name)
        self.publish_message("data-corner-ready", finished_ellipse_message)
        print "Start producing gradient videos"
        self.generate_gradient_videos(new_added_vids_name)

    def get_consumers(self, Consumer, channel):
        return [Consumer(queues=self.queues, callbacks=self.callbacks)]


def main():
    ex = Exchange('editing_exchange', type='direct')
    queue_ready = Queue('', exchange=ex, routing_key='data-file-ready')
    queues = [queue_ready, ]
    with Connection('amqp://guest:guest@localhost:5672//') as connection:
        try:
            da_video_worker = DAvideoWorker(connection, queues)
            print "start the Deep Analysis Video Worker"
            da_video_worker.run()
        except KeyboardInterrupt:
            sys.exit(0)


if __name__ == '__main__':
    main()

