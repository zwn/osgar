import unittest
from unittest.mock import MagicMock
import math

from osgar.drivers.rosmsg import ROSMsgParser, parse_image


class ROSMsgParserTest(unittest.TestCase):

    def Xtest_parse_imu(self):
        r = ROSMsgParser(config={}, bus=None)
        with open('imu_data.txt', 'rb') as f:
            r._buf += f.read()
            index = 0
            packet = r.get_packet()  # first packet is structure file
            while packet is not None:
                if index > 0:
                    ori = r.parse(packet)
                    q0, q1, q2, q3 = ori  # quaternion
                    
                    x =  math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))
                    y =  math.asin(2*(q0*q2-q3*q1))
                    z =  math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
#                    print([math.degrees(a) for a in [x, y, z]])

                packet = r.get_packet()
                index += 1
#                if index > 10000:
#                    break

    def test_parse_image(self):
        r = ROSMsgParser(config={}, bus=None)
        with open('image_raw.bin', 'rb') as f:
            r._buf += f.read()
            index = 0
            packet = r.get_packet()  # first packet is structure file
            while packet is not None:
                if index > 0:
                    parse_image(packet, 'dump.ppm')
                packet = r.get_packet()
                index += 1
                if index > 10:
                    break



# vim: expandtab sw=4 ts=4
