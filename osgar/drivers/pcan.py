"""
  Wrapper for PeakCAN USB device
"""
#
# pip install python-can
# https://python-can.readthedocs.io/en/2.1.0/interfaces/pcan.html
#
# https://www.peak-system.com/
# https://www.peak-system.com/fileadmin/media/files/pcan-basic.zip
# http://www.peak-system.com/quick/BasicLinux
#
# https://en.wikipedia.org/wiki/CAN_bus
#
# The new CAN messages will contain arbitration_id, data and flags
#  http://docs.ros.org/melodic/api/can_msgs/html/msg/Frame.html
#

import struct

import can
from threading import Thread

from osgar.bus import BusShutdownException

IS_EXTENDED_ID_MASK = 0x1


class OldPeakCAN:
    def __init__(self, config, bus):
        self.bus = bus
        self.canbus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=500000)
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.output_thread = Thread(target=self.run_output, daemon=True)

    def start(self):
        self.input_thread.start()
        self.output_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)
        self.output_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            msg = self.canbus.recv()  # TODO timeout
            flags = IS_EXTENDED_ID_MASK if msg.is_extended_id else 0
            self.bus.publish('can', [msg.arbitration_id, msg.data, flags])

    def slot_raw(self, timestamp, packet):
        arbitration_id, data, flags = packet
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=(flags & IS_EXTENDED_ID_MASK))
        self.canbus.send(msg)  # TODO timeout, locks?!

    def run_output(self):
        try:
            while True:
                dt, __, data = self.bus.listen()
                self.slot_raw(dt, data)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

##################################################
from multiprocessing import Process, Queue

def my_process(in_queue, out_queue):
    print('my_process started')
    canbus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=500000)
    while True:
        msg = canbus.recv()  # TODO timeout
        flags = IS_EXTENDED_ID_MASK if msg.is_extended_id else 0
        out_queue.put([msg.arbitration_id, msg.data, flags])

        if not in_queue.empty():
            packet = in_queue.get()
            if packet is None:
                break
            arbitration_id, data, flags = packet
            msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=(flags & IS_EXTENDED_ID_MASK))
            canbus.send(msg)  # TODO timeout, locks?!
    print('my_process terminated')


class PeakCAN:
    def __init__(self, config, bus):
        self.bus = bus
        self.q_out = Queue()
        self.q_in = Queue()
        self.process = Process(target=my_process, args=(self.q_out, self.q_in,))
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.output_thread = Thread(target=self.run_output, daemon=True)

    def start(self):
        self.process.start()
        self.input_thread.start()
        self.output_thread.start()

    def join(self, timeout=None):
        self.process.join(timeout=timeout)
        self.input_thread.join(timeout=timeout)
        self.output_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            msg = self.q_in.get()
            self.bus.publish('can', msg)

    def slot_raw(self, timestamp, packet):
        self.q_out.put(packet)
        #arbitration_id, data, flags = packet

    def run_output(self):
        try:
            while True:
                dt, __, data = self.bus.listen()
                self.slot_raw(dt, data)
        except BusShutdownException:
            self.q_out.put(None)

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
