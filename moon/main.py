"""
  Space Robotics Challenge 2
"""

import math
from datetime import timedelta

from osgar.node import Node
from osgar.bus import BusShutdownException


def distance(pose1, pose2):
    return math.hypot(pose1[0] - pose2[0], pose1[1] - pose2[1])


class SpaceRoboticsChallenge(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("desired_speed", "artf_xyz", "pose3d", "request_origin")
        self.last_position = (0, 0, 0)  # proper should be None, but we really start from zero
        self.max_speed = 1.0  # oficial max speed is 1.5m/s

    def send_speed_cmd(self, speed, angular_speed):
        self.bus.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    def go_straight(self, how_far, timeout=None):
        print(self.time, "go_straight %.1f (speed: %.1f)" % (how_far, self.max_speed), self.last_position)
        start_pose = self.last_position
        if how_far >= 0:
            self.send_speed_cmd(self.max_speed, 0.0)
        else:
            self.send_speed_cmd(-self.max_speed, 0.0)
        start_time = self.time
        while distance(start_pose, self.last_position) < abs(how_far):
            self.update()
            if timeout is not None and self.time - start_time > timeout:
                print("go_straight - TIMEOUT!")
                break
        self.send_speed_cmd(0.0, 0.0)

    def run(self):
        try:
            self.go_straight(10.0, timeout=timedelta(seconds=60))
        except BusShutdownException:
            pass


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Space Robotics Challenge 2')
    args = parser.parse_args()

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
