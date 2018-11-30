"""
  ROS (Robot Operating System) Proxy
"""

from threading import Thread
import struct
from xmlrpc.client import ServerProxy
from xmlrpc.server import SimpleXMLRPCServer

import socket


from osgar.bus import BusShutdownException

NODE_HOST, NODE_PORT = ('127.0.0.1', 8000)
PUBLISH_PORT = 8123


ROS_MESSAGE_TYPES = {
    'std_msgs/String': '992ce8a1687cec8c8bd883ec73ca41d1',
    'geometry_msgs/Twist': '9f195f881246fdfa2798d1d3eebca84a',
    'std_msgs/Imu': '6a62c6daae103f4ff57a132d6f95cec2',
}


def prefix4BytesLen(s):
    "adding ROS length"
    if type(s) == str:
        s = bytes(s, encoding='ascii')
    return struct.pack("I", len(s)) + s


def packCmdVel(speed, angularSpeed):
    return struct.pack("dddddd", speed,0,0, 0,0,angularSpeed)


def publisherUpdate(caller_id, topic, publishers):
    print("called 'publisherUpdate' with", (caller_id, topic, publishers))
    return (1, "Hi, I am fine", 42) # (code, statusMessage, ignore) 

def requestTopic(caller_id, topic, publishers):
    print("REQ", (caller_id, topic, publishers))
    return 1, "ready on martind-blabla", ['TCPROS', NODE_HOST, PUBLISH_PORT]


class ROSMsgParser(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus

    def get_packet(self):
        pass

    def parse(self, data):
        pass

    def run(self):
        try:
            while True:
                timestamp, channel, data = self.bus.listen()
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
