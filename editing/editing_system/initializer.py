#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : initializer.py
 #Creation Date : 08-09-2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.

"""
Code for Initializer module in editing system 
"""

#TODO: Implement Loggin for this
from kombu import Connection, Exchange, Queue
from kombu.mixins import ConsumerMixin 
import message_publisher
import sys


class Initializer(ConsumerMixin):
    
    def __init__(self, connection, queues, callbacks=None):
        """
        keyword arguments:
        queues -- a list of queues the consumer will subscribe
        """
        self.connection = connection 
        self.queues = queues 
        if not callbacks:
            self.callbacks = [self.process_message,]
        self.initialize_publish()
        

    def initialize_publish(self): 
        # We may end up not using this method 
        exchange = Exchange('editing_exchange', type='direct')
        routing_key = "data-start-initialize"
        initialize_publisher = message_publisher.Message_publisher(self.connection, exchange, 
                routing_key)
        #Initializer will tell filewatcher how many files to watch
        msg = dict(
                enough_quantity = 5,
                ) 
        initialize_publisher.publishMessage(msg)


    def get_consumers(self, Consumer, channel):
        return [Consumer(queues=self.queues, callbacks=self.callbacks)]
        

    def process_message(self, body, message):    
        if(message.delivery_info['routing_key'] == "data-start-enough"):
            #TODO: We may need some logic here for the robot or change the metric
            print "get enough data"
        message.ack()


def main():
    ex = Exchange('editing_exchange', type='direct')
    queue_start_enough = Queue('', exchange=ex, routing_key='data-start-enough')
    queue_initialized = Queue('', exchange=ex, routing_key='data-initialized')
    queues = [queue_start_enough, queue_initialized]
    #Let's first work on default address
    with Connection('amqp://guest:guest@localhost:5672//') as connection:
        try: 
            initializer = Initializer(connection, queues)
            print "start the initializer"
            initializer.run()
        except KeyboardInterrupt:
            sys.exit(0)


if __name__=='__main__':
    main()
