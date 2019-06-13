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
CAN_ID_CURRENT = 0x10


def draw(arr):
    import matplotlib.pyplot as plt
    plt.plot(range(len(arr)), arr, '-', linewidth=2)
    plt.show()


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

        self.verbose = False  # should be in Node
        self.enc_debug_arr = []

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
#            print(rpm3, current, duty_cycle)
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
            elif msg_id == CAN_ID_CURRENT:
                assert len(packet) == 5, len(packet)  # expected 24bit integer miliAmps
                current = struct.unpack_from('>i', packet, 1)[0] & 0xFFFFFF
#                print(current)

            if msg_id == CAN_ID_SYNC:
                self.publish('encoders', 
                        [self.last_encoders_front_left, self.last_encoders_front_right,
                         self.last_encoders_rear_left, self.last_encoders_rear_right])
                if self.update_pose():
                    self.send_pose()
                # reset all encoder values to be sure that new reading were received
#                self.last_encoders_front_left = None
#                self.last_encoders_front_right = None
#                self.last_encoders_rear_left = None
#                self.last_encoders_rear_right = None
                return True
        return False

    def slot_can(self, data):
        limit_brake = 100
        if abs(self.desired_angular_speed) < math.radians(5):
            limit_r = 400
            limit_l = 400
        elif self.desired_angular_speed > 0:  # TODO handle backup
            # turn left
            limit_r = 400
            limit_l = 200
        else:
            # turn right
            limit_r = 200
            limit_l = 400
        if self.process_packet(data):
#            print(self.last_encoders_front_left, self.last_encoders_front_right)
            fwd = [0, 0, 72, 0]  # 6Amp
            bwd = [255, 255, 255-72, 255]  # 4Amp
            stop = [0, 0, 0, 0]
            if self.verbose:
                cmd_l, cmd_r = limit_l, limit_r
                if self.desired_speed < 0:
                    cmd_l, cmd_r = -cmd_l, -cmd_r
                elif self.desired_speed == 0:
                    cmd_l, cmd_r = 0, 0
                self.enc_debug_arr.append((cmd_l, cmd_r,
                    self.last_encoders_front_left, self.last_encoders_front_right,
                    self.last_encoders_rear_left, self.last_encoders_rear_right))
            if self.desired_speed > 0:
                if self.last_encoders_front_right is not None:
                    self.publish('can', CAN_packet(0x31, [0, 0, limit_r//256, limit_r % 256]))
                    """
                    if abs(self.last_encoders_front_right) > limit_r:
                        if abs(self.last_encoders_front_right) - limit_r > limit_brake:
                            self.publish('can', CAN_packet(0x21, stop))  # brake right front
                        else:
                            self.publish('can', CAN_packet(0x11, stop))  # right front
                    else:
                        self.publish('can', CAN_packet(0x11, fwd))  # right front"""
                if self.last_encoders_front_left is not None:
                    self.publish('can', CAN_packet(0x32, [0, 0, limit_l//256, limit_l % 256]))
                    """
                    if abs(self.last_encoders_front_left) > limit_l:
                        if abs(self.last_encoders_front_left) - limit_l > limit_brake:
                            self.publish('can', CAN_packet(0x22, stop))  # brake left front
                        else:
                            self.publish('can', CAN_packet(0x12, stop))  # left front
                    else:
                        self.publish('can', CAN_packet(0x12, fwd))  # left front"""
                self.publish('can', CAN_packet(0x13, stop))  # right rear
                self.publish('can', CAN_packet(0x14, stop))  # left rear

            elif self.desired_speed < 0:
                if self.last_encoders_rear_right is not None:
                    self.publish('can', CAN_packet(0x33, [255, 255, 255 - limit_r//256, 255 - limit_r % 256]))
                    """
                    if abs(self.last_encoders_rear_right) > limit_r:
                        self.publish('can', CAN_packet(0x13, stop))  # right front
                    else:
                        self.publish('can', CAN_packet(0x13, bwd))  # right front"""
                if self.last_encoders_rear_left is not None:
                    self.publish('can', CAN_packet(0x34, [255, 255, 255 - limit_l//256, 255 - limit_l % 256]))
                    """
                    if abs(self.last_encoders_rear_left) > limit_l:
                        self.publish('can', CAN_packet(0x14, stop))  # left front
                    else:
                        self.publish('can', CAN_packet(0x14, bwd))  # left front"""
                self.publish('can', CAN_packet(0x11, stop))  # right front
                self.publish('can', CAN_packet(0x12, stop))  # left front 

            else:
                self.publish('can', CAN_packet(0x11, stop))  # right front
                self.publish('can', CAN_packet(0x12, stop))  # left front
                self.publish('can', CAN_packet(0x13, stop))  # right rear
                self.publish('can', CAN_packet(0x14, stop))  # left rear


    def slot_desired_speed(self, data):
        self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)

    def run(self):
        try:
            while True:
                try:
                    self.time, channel, data = self.listen()
                except StopIteration:
                    if self.verbose:
                        print(len(self.enc_debug_arr))
                    raise BusShutdownException

                if channel == 'can':
                    self.slot_can(data)
                elif channel == 'desired_speed':
                    self.slot_desired_speed(data)
                else:
                    assert False, channel  # unsupported channel
        except BusShutdownException:
            pass

    def draw(self):
        """
        Debug - draw encoders
        """
        draw(self.enc_debug_arr)

# vim: expandtab sw=4 ts=4
