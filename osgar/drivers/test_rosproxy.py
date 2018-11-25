import unittest
from unittest.mock import patch, MagicMock
import threading
from threading import Thread
import os
from xmlrpc.server import SimpleXMLRPCServer
import xmlrpc
import time

from osgar.drivers.rosproxy import ROSProxy
from osgar.bus import BusHandler


# TODO refactor into common testing code
# https://stackoverflow.com/questions/12484175/make-python-unittest-fail-on-exception-from-any-thread
class GlobalExceptionWatcher(object):
    def _store_excepthook(self):
        '''
        Uses as an exception handlers which stores any uncaught exceptions.
        '''
        formated_exc = self.__org_hook()
        self._exceptions.append(formated_exc)
        return formated_exc

    def __enter__(self):
        '''
        Register us to the hook.
        '''
        self._exceptions = []
        self.__org_hook = threading._format_exc
        threading._format_exc = self._store_excepthook

    def __exit__(self, type, value, traceback):
        '''
        Remove us from the hook, assure no exception were thrown.
        '''
        threading._format_exc = self.__org_hook
        if len(self._exceptions) != 0:
            tracebacks = os.linesep.join(self._exceptions)
            raise Exception('Exceptions in other threads: %s' % tracebacks)

def getSystemState(path):
    print("RECEIVED", path)
    return (1, 0, ([], 0, 0))

class DummyROSMaster(Thread):
    def __init__(self, nodeAddrHostPort ):
            #ros_master_uri):
        Thread.__init__( self )
        self.setDaemon( True )
        self.server = SimpleXMLRPCServer( nodeAddrHostPort )
        print("Listening on port %d ..." % nodeAddrHostPort[1])
        self.server.register_function(getSystemState, "getSystemState")
#        self.server.register_function(requestTopic, "requestTopic")
        self.start()

    def run( self ):
        self.server.serve_forever()


class ROSProxyTest(unittest.TestCase):

    def test_usage(self):
        logger = MagicMock()
        bus = BusHandler(logger)
        config = {
                'ros_master_uri': 'http://127.0.0.1:11311',
                'ros_client_uri': 'http://127.0.0.1:8000',
                'topic': '/hello',
                'topic_type': 'std_msgs/String',
                }

        master = DummyROSMaster(('127.0.0.1', 11311)) #8000)) #11311))  #config['ros_master_uri'])
        print('time to sleep')
        time.sleep(1.0)
        print('start client')
#        s = xmlrpc.client.ServerProxy('http://127.0.0.1:8000')
#        s = xmlrpc.client.ServerProxy('http://127.0.0.1:11311')
#        print(s.getSystemState('/'))
#        print(s.listMethods())
#        return
        
        proxy = ROSProxy(config=config, bus=bus)
        with GlobalExceptionWatcher():
            proxy.start()
            proxy.join()

# vim: expandtab sw=4 ts=4
