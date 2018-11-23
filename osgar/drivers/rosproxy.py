"""
  ROS (Robot Operating System) Proxy
"""

from threading import Thread
import struct

from osgar.bus import BusShutdownException


class ROSProxy(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus

    def run(self):
        try:
            while True:
                packet = self.bus.listen()
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
