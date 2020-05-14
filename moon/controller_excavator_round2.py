"""
  Space Robotics Challenge 2
"""
import zmq

from moon.controller import SpaceRoboticsChallenge

class SpaceRoboticsChallengeExcavatorRound2(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)

    def run(self):

        self.socket_out.send_string('get_volatile_locations')
        vol_list = self.socket_out.recv().decode('ascii')
        print (self.time, "main-excavator-round2: Volatiles: %s" % vol_list)

        super().run()

# vim: expandtab sw=4 ts=4
