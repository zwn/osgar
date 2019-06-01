"""
  Driver for articulated robot Kloubak
  (https://github.com/tf-czu/kloubak)
"""

import ctypes
import struct
import math
from datetime import timedelta

from .canserial import CAN_packet
from osgar.node import Node
from osgar.bus import BusShutdownException

WHEEL_DISTANCE = 0.475  # m
VESC_REPORT_FREQ = 100  # Hz
ENC_SCALE = 0.25 * math.pi / (4 * 3 * 60 * VESC_REPORT_FREQ)  # scale 4x found experimentally

CAN_ID_BUTTONS = 0x1
CAN_ID_VESC_FRONT_R = 0x91
CAN_ID_VESC_FRONT_L = 0x92
CAN_ID_VESC_REAR_R = 0x93
CAN_ID_VESC_REAR_L = 0x94
CAN_ID_SYNC = CAN_ID_VESC_FRONT_L


class RobotKloubak(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

        # commands
        self.desired_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0

        # status
        self.emergency_stop = None  # uknown state
        self.pose = (0.0, 0.0, 0.0)  # x, y in meters, heading in radians (not corrected to 2PI)
        self.buttons = None
        self.last_encoders_front_left = None
        self.last_encoders_front_right = None
        self.last_encoders_rear_left = None
        self.last_encoders_rear_right = None
        self.last_encoders_time = None

    def send_pose(self):
        x, y, heading = self.pose
        self.publish('pose2d', [round(x*1000), round(y*1000),
                                round(math.degrees(heading)*100)])

    def update_buttons(self, data):
        assert len(data) == 1, len(data)
        val = data[0]
        if self.buttons is None or val != self.buttons:
            self.buttons = val
            stop_status = self.buttons & 0x01 == 0x01
            if self.emergency_stop != stop_status:
                self.emergency_stop = stop_status
                self.bus.publish('emergency_stop', self.emergency_stop)
                print('Emergency STOP:', self.emergency_stop)

    def update_encoders(self, msg_id, data):
        assert len(data) == 8, data
        rpm3, current, duty_cycle = struct.unpack('>ihh', data)
        if msg_id == CAN_ID_VESC_FRONT_L:
            self.last_encoders_front_left = rpm3
        elif msg_id == CAN_ID_VESC_FRONT_R:
            self.last_encoders_front_right = rpm3
        if msg_id == CAN_ID_VESC_REAR_L:
            self.last_encoders_rear_left = rpm3
        elif msg_id == CAN_ID_VESC_REAR_R:
            self.last_encoders_rear_right = rpm3

    def update_pose(self):
        """Update internal pose with 'dt' step"""
        if self.last_encoders_front_left is None or self.last_encoders_front_right is None:
            return False
        x, y, heading = self.pose

        metricL = ENC_SCALE * self.last_encoders_front_left  # dt is already part of ENC_SCALE
        metricR = ENC_SCALE * self.last_encoders_front_right

        dist = (metricL + metricR)/2.0
        angle = (metricR - metricL)/WHEEL_DISTANCE

        # advance robot by given distance and angle
        if abs(angle) < 0.0000001:  # EPS
            # Straight movement - a special case
            x += dist * math.cos(heading)
            y += dist * math.sin(heading)
            #Not needed: heading += angle
        else:
            # Arc
            r = dist / angle
            x += -r * math.sin(heading) + r * math.sin(heading + angle)
            y += +r * math.cos(heading) - r * math.cos(heading + angle)
            heading += angle # not normalized
        self.pose = (x, y, heading)
        return True

    def process_packet(self, packet, verbose=False):
        if len(packet) >= 2:
            msg_id = ((packet[0]) << 3) | (((packet[1]) >> 5) & 0x1f)
#            print(hex(msg_id), packet[2:])
            if msg_id == CAN_ID_BUTTONS:
                self.update_buttons(packet[2:])
            elif msg_id in [CAN_ID_VESC_FRONT_L, CAN_ID_VESC_FRONT_R, CAN_ID_VESC_REAR_L, CAN_ID_VESC_REAR_R]:
                self.update_encoders(msg_id, packet[2:])

            if msg_id == CAN_ID_SYNC:
                self.publish('encoders', 
                        [self.last_encoders_front_left, self.last_encoders_front_right,
                         self.last_encoders_rear_left, self.last_encoders_rear_right])
                if self.update_pose():
                    self.send_pose()
                # reset all encoder values to be sure that new reading were received
                self.last_encoders_front_left = None
                self.last_encoders_front_right = None
                self.last_encoders_rear_left = None
                self.last_encoders_rear_right = None
                return True
        return False

    def slot_can(self, data):
        use_current = True
        if self.process_packet(data):
            if self.desired_speed > 0:
                if use_current:
                    cmd = [0, 0, 4, 0]  # 1Amp
                    self.publish('can', CAN_packet(0x11, cmd))  # right front
                    self.publish('can', CAN_packet(0x12, cmd))  # left front
#                    self.publish('can', CAN_packet(0x13, cmd))  # right rear
#                    self.publish('can', CAN_packet(0x14, cmd))  # left rear
                else:
                    cmd = [0, 0, 0, 60]
                    self.publish('can', CAN_packet(0x31, cmd))  # right front
                    self.publish('can', CAN_packet(0x32, cmd))  # left front
                    self.publish('can', CAN_packet(0x33, cmd))  # right rear
                    self.publish('can', CAN_packet(0x34, cmd))  # left rear
            else:
                if use_current:
                    cmd = [0, 0, 0, 0]
                    self.publish('can', CAN_packet(0x11, cmd))  # right front
                    self.publish('can', CAN_packet(0x12, cmd))  # left front
                    self.publish('can', CAN_packet(0x13, cmd))  # right rear
                    self.publish('can', CAN_packet(0x14, cmd))  # left rear
                else:
                    self.publish('can', CAN_packet(0x21, [0, 0, 0, 0]))  # right front
                    self.publish('can', CAN_packet(0x22, [0, 0, 0, 0]))  # left front
                    self.publish('can', CAN_packet(0x23, [0, 0, 0, 0]))  # right rear
                    self.publish('can', CAN_packet(0x24, [0, 0, 0, 0]))  # left rear

    def slot_desired_speed(self, data):
        self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)

    def run(self):
        try:
            while True:
                self.time, channel, data = self.listen()
                if channel == 'can':
                    self.slot_can(data)
                elif channel == 'desired_speed':
                    self.slot_desired_speed(data)
                else:
                    assert False, channel  # unsupported channel
        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
