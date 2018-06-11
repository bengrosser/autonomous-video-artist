from kombu import Connection, Exchange, Queue
from kombu.mixins import ConsumerMixin
import message_publisher
import sys
import sqlite3
from range_finder import range_finder, range_filter


#TODO: pulish the message to the exchange
class MetadataWorker(ConsumerMixin):

    def __init__(self, connection, queues, callbacks=None):
        self.connection = connection
        self.queues = queues
        if not callbacks:
            self.callbacks = [self.process_message, ]

    @staticmethod
    def _update_metadata_score(vids_names):
        connection = sqlite3.connect('Editing.db')
        cursor = connection.cursor()
        for vid_name in vids_names:
            cursor.execute('SELECT avg_brightness, avg_distance, motion_detected, has_obstacle, battery_level, '
                           'distance_to_dock, RAM_in_use_sys FROM Metadata WHERE file_name=?', (vid_name,))
            vid_metadata_list = cursor.fetchall()
            if len(vid_metadata_list[0]) != 7:
                raise Exception('Not enough field for ' + vid_name)
            else:
                avg_brightness = vid_metadata_list[0][0]
                avg_distance = vid_metadata_list[0][1]
                motion_detected = vid_metadata_list[0][2]
                has_obstacle = vid_metadata_list[0][3]
                battery_level = vid_metadata_list[0][4]
                distance_to_dock = vid_metadata_list[0][5]
                RAM_in_use_sys = vid_metadata_list[0][6]
                # Start to normalise the data
                avg_brightness_distance_to_128 = abs(avg_brightness-128)
                normalized_avg_brightness = float(128 - avg_brightness_distance_to_128) / 128
                if avg_distance >= 1:
                    normalized_avg_distance = 1
                else:
                    normalized_avg_distance = avg_distance
                normalized_motion_detected = motion_detected
                normalized_has_obstacle = has_obstacle
                normalized_battery_level = float(battery_level)/162
                if distance_to_dock >= 5:
                    normalized_distance_to_dock = 0
                else:
                    normalized_distance_to_dock = float(5 - distance_to_dock) / 5

                storage_converter_unit = 2*1024*1024*1024
                RAM_in_gb = RAM_in_use_sys / float(storage_converter_unit)
                RAM_distance_from_1 = abs(RAM_in_gb-1)
                normalized_RAM_in_use_sys = (1 - RAM_distance_from_1) / 1
                print "normalized avg brightness " + str(normalized_avg_brightness)
                print "normalized avg distance " + str(normalized_avg_distance)
                print "normalized motion detection "+ str(normalized_motion_detected)
                print "normalized has obstacle " + str(normalized_has_obstacle)
                print "normalized battery level " + str(normalized_battery_level)
                print "normalized distance to dock " + str(normalized_distance_to_dock)
                print "normalized ram in use " + str(normalized_RAM_in_use_sys)
                metadata_score = 2*normalized_avg_brightness + normalized_avg_distance + 2*normalized_has_obstacle \
                                 + 2*normalized_motion_detected + 2*normalized_battery_level \
                                 + 2*normalized_RAM_in_use_sys + 2*normalized_distance_to_dock
                cursor.execute('UPDATE Metadata SET metadata_score=? WHERE file_name=?', (metadata_score, vid_name))
                connection.commit()
        connection.close()

    @staticmethod
    def _update_editing_range(vids_names):
        connection = sqlite3.connect('Editing.db')
        cursor = connection.cursor()
        for vid_name in vids_names:
            cursor.execute('SELECT file_path, battery_level FROM Metadata WHERE file_name=?', (vid_name,))
            file_path, battery_level = cursor.fetchall()[0]
            editing_range = range_finder(file_path, battery_level, 0.7)
            editing_range = range_filter(editing_range)
            editing_range_string = str(editing_range)
            print "Editing Range is", editing_range_string
            cursor.execute('UPDATE Metadata SET to_edit=? WHERE file_name=?', (editing_range_string, vid_name))
            connection.commit()
        connection.close()

    def process_message(self, body, message):
        new_added_vids_name = body["ready_files"]
        print "Got the message, start to update metadata scores"
        self._update_metadata_score(new_added_vids_name)
        print "Finish score update"
        self._update_editing_range(new_added_vids_name)

    def publish_message(self, routing_key, message):
        exchange = Exchange('editing_exchange', type='direct')
        file_ready_publisher = message_publisher.Message_publisher(self.connection, exchange,
                                                                   routing_key)
        file_ready_publisher.publishMessage(message)

    def get_consumers(self, Consumer, channel):
        return [Consumer(queues=self.queues, callbacks=self.callbacks)]


def main():
    ex = Exchange('editing_exchange', type='direct')
    queue_ready = Queue('', exchange=ex, routing_key='data-file-ready')
    queues = [queue_ready, ]
    # Let's first work on default address
    with Connection('amqp://guest:guest@localhost:5672//') as connection:
        try:
            metadata_worker = MetadataWorker(connection, queues)
            print "start the MetaData worker"
            metadata_worker.run()
        except KeyboardInterrupt:
            sys.exit(0)


if __name__ == '__main__':
    main()