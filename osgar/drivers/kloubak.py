"""
  Driver for articulated robot Kloubak
  (https://github.com/tf-czu/kloubak)
"""

import ctypes
import struct
import math
from datetime import timedelta

from osgar.node import Node
from osgar.bus import BusShutdownException


class RobotKloubak(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

        # commands
        self.desired_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0

        # status
        self.emergency_stop = None  # uknown state
        self.pose = (0.0, 0.0, 0.0)  # x, y in meters, heading in radians (not corrected to 2PI)
        self.flags = None
        self.last_encoders = None

    def send_pose(self):
        x, y, heading = self.pose
        self.publish('pose2d', [round(x*1000), round(y*1000),
                                round(math.degrees(heading)*100)])

    def run(self):
        try:
            while True:
                dt, channel, data = self.listen()
                self.time = dt
                if channel == 'can':
                    if self.desired_speed > 0:
                        self.publish('can', CAN_packet(0x11, [0, 0, 8, 108]))  # right front
                        self.publish('can', CAN_packet(0x12, [0, 0, 8, 108]))  # left front
                    else:
                        self.publish('can', CAN_packet(0x11, [0, 0, 0, 0]))  # right front
                        self.publish('can', CAN_packet(0x12, [0, 0, 0, 0]))  # left front

                if channel == 'desired_speed':
                    self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)                    
        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
