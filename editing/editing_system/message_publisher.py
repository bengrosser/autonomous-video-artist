#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : message_publisher.py
 #Creation Date : 08-09-2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.

'''
A helper class for publishing messages to 
exchange
'''

class Message_publisher(object):
    def __init__(self, connection, exchange, routing_key):
        self.connection = connection
        self.exchange = exchange
        self.routing_key= routing_key
        self.producer = connection.Producer(connection, exchange=exchange, 
                                                    routing_key=routing_key)


    def publishMessage(self, msg):
        self.producer.publish(msg, self.routing_key)
