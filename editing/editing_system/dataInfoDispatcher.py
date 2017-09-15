"""
"""

import sys
import argparse
import logging
import topolens
import mq_DataAvail
from kombu import Connection, Exchange, Queue
from kombu.mixins import ConsumerMixin
from topolens import Catalog, add_gs_args, add_mq_args


class dispatch_Consumer(ConsumerMixin):
    def __init__(self, connection, queues, gs, callbacks=None):
        self.connection = connection
        self.queues=queues
        self.gs = gs 
        if not callbacks:
            self.callbacks = [self.process_messsage,]

    
    def get_consumers(self, Consumer, channel):
        return [Consumer(queues=self.queues, callbacks=self.callbacks)]
    
    
    def process_messsage(self, body, message):
        process_type = body['type']        
        process_id = body['id']
        self.geo_process(process_id, process_type) 
    

    def send_data_ready(self, process_id, process_type):
        dataReadyLogger = dataMQLogger(self.connection)
        msg = dataReadyLogger.message(process_id, process_type) 
        dataReadyLogger.publicMessage(msg)


    def geo_process(self, process_id, process_type):
        layer_name = process_id + '_' + process_type
        vizOutputName = process_type.lower()
        if vizOutputName == 'dem':
            vizOutputName = 'dem_raw'
        vizOutputName = vizOutputName + '.tif'
        self.gs.create_imagepyramid(layer_name, os.path.join(process_id, layer_name), 
                mode='external', overwrite=True)
        VizSLDGen(process_id, process_type, vizOutputName, '.')
        with open(layer_name+'.sld') as f:
            self.gs.create_style(layer_name, f.read(), 
                    workspace=self.gs.workspace, overwrite=True)
        self.gs.apply_style(layer_name, layer_name)
        self.send_data_ready(process_id, process_type) 
        
        
def main():
    parser = argparse.ArgumentParser()
    add_mq_args(parser)
    args = parser.parse_args()
    ex = Exchange('topolens', type='topic')
    queue = Queue('', exchange=ex, routing_key='data.avail')

    geoserver = Catalog(args.gs_service_url, args.gs_login, args.gs_password)
    geoserver.workspace = args.gs_workspace
    geoserver.external_path = args.gs_remote_jobdir
    
    
    with Connection(hostname=args.mq_hostname, port=args.mq_port,
            userid=args.mq_login, password=args.mq_password,
            virtual_host=args.mq_virtual_host) as connection:
        print connecton.as_uri()

        try:
            myDispatcher = dispatch_Consumer(connecton, queue, geoserver)
            myDispatcher.run()

        except KeyboardInterrupt:
            sys.exit(0)

if __name__ == '__main__':
    main()
