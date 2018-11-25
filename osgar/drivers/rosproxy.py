"""
  ROS (Robot Operating System) Proxy
"""

from threading import Thread
import struct
from xmlrpc.client import ServerProxy

from osgar.bus import BusShutdownException


ROS_MESSAGE_TYPES = {
    'std_msgs/String': '992ce8a1687cec8c8bd883ec73ca41d1',
    'geometry_msgs/Twist': '9f195f881246fdfa2798d1d3eebca84a',
}


class ROSProxy(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus
        self.ros_master_uri = config['ros_master_uri']
        self.ros_client_uri = config['ros_client_uri']
        self.topic = config['topic']
        self.topic_type = config['topic_type']

    def run(self):
        master = ServerProxy(self.ros_master_uri)
        code, status_message, system_state = master.getSystemState('/')
        assert code == 1, code
        assert len(system_state) == 3, system_state
        print("Publishers:")
        for s in system_state[0]:
            print(s)

        caller_id = "/osgar_node"
        code, status_message, subscribers = master.registerPublisher(
                caller_id, self.topic, self.topic_type, self.ros_client_uri)

        print(subscribers)

        try:
            while True:
                packet = self.bus.listen()
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
