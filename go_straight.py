"""
Go straight!
"""
import math
import time
from datetime import timedelta

from osgar.node import Node
from osgar.drivers.canserial import CAN_packet


class Go_straight(Node):
    def __init__(self, config, bus):
        print("init")
        super().__init__(config, bus)
        self.verbose = False


    def update(self):
        channel = super().update()  # define self.time
        return channel


    def run(self):
        self.update()
        print(self.time, '=== ver0 ===')
        #self.publish('can', CAN_packet(0x0, [1, 0]))
        game_start = self.time
        while self.time - game_start < timedelta(seconds=5):
            self.publish('can', CAN_packet(0x11, [0, 0, 8, 108]))  # right front
            self.publish('can', CAN_packet(0x12, [0, 0, 8, 108]))  # left front
            print([hex(b) for b in self.can])
            self.update()
        #self.publish('can', CAN_packet(0x0, [128, 0]))
        self.publish('can', CAN_packet(0x11, [0, 0, 0, 0]))  # right front
        self.publish('can', CAN_packet(0x12, [0, 0, 0, 0]))  # left front


if __name__ == "__main__":
    from osgar.launcher import launch

    launch(app=Go_straight, description='record data', prefix='go_straight-')
