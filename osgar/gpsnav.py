"""
  GPS Waypoint Navigation  (target RoboOrienteering/Magellan contest)
"""

import argparse
import sys
import math
from datetime import timedelta
from queue import Queue

from osgar.logger import LogWriter, LogReader
from osgar.lib.config import load as config_load
from osgar.lib.mathex import normalizeAnglePIPI
from osgar.drivers import all_drivers
from osgar.record import Recorder

from osgar.drivers.gps import INVALID_COORDINATES
from osgar.bus import BusHandler


def geo_length(pos1, pos2):
    "return distance on sphere for two integer positions in milliseconds"
    x_scale = math.cos(math.radians(pos1[0]/3600000))
    scale = 40000000/(360*3600000)
    return math.hypot((pos2[0] - pos1[0])*x_scale, pos2[1] - pos1[1]) * scale


def geo_angle(pos1, pos2):
    if geo_length(pos1, pos2) < 1.0:
        return None
    x_scale = math.cos(math.radians(pos1[0]/3600000))
    return math.atan2(pos2[1] - pos1[1], (pos2[0] - pos1[0])*x_scale)


def latlon2xy(lat, lon):
    return int(round(lon*3600000)), int(round(lat*3600000))

class EmergencyStopException(Exception):
    pass

class EmergencyStopMonitor:
    def __init__(self, robot):
        self.robot = robot

    def update(self, robot):
        if robot.status is not None and (robot.status & GPSNav.EMERGENCY_STOP) == 0:
            raise EmergencyStopException()

    # context manager functions
    def __enter__(self):
        self.callback = self.robot.register(self.update)
        self.robot.raise_exception_on_stop = True
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.callback)
        self.robot.raise_exception_on_stop = False


class GPSNav:
    EMERGENCY_STOP = 0x0001

    def __init__(self, config, bus):
        self.bus = bus
        self.maxspeed = config['maxspeed']
        self.goals = [latlon2xy(lat, lon) for lat, lon in config['waypoints']]
        self.time = None
        self.last_gps_position = None  # (lon, lat) in milliseconds
        self.last_imu_yaw = None  # magnetic north in degrees
        self.status = None
        self.wheel_heading = None
        self.cmd = [0, 0]
        self.monitors = []
        self.last_gps_position_angle = None  # for angle computation from dGPS

        self.last_pose2d = (0, 0, 0)
        self.raise_exception_on_stop = False

    def update(self):
        packet = self.bus.listen()
        if packet is not None:
#            print('RO', packet)
            timestamp, channel, data = packet
            self.time = timestamp
            if channel == 'position':
                self.last_gps_position = data
            elif channel == 'orientation':
                (yaw, pitch, roll), (magx, y, z), (accx, y, z), (gyrox, y, z) = data
                self.last_imu_yaw = yaw
            elif channel == 'status':  # i.e. I can drive only spider??
                self.status, steering_status = data
                if steering_status is None:
                    self.wheel_heading = None
                else:
                    self.wheel_heading = math.radians(-360 * steering_status[0] / 512)
                self.bus.publish('move', self.cmd)
            elif channel == 'pose2d':
                x, y, heading = data
                self.last_pose2d = [x/1000.0, y/1000.0, math.radians(heading/100.0)]
                self.wheel_heading = heading  # for Eduro it is direction of the robot
                desired_speed = 0 if self.cmd[0] == 0 else 0.2
#                desired_angular_speed = math.radians(self.cmd[1])  # TODO check sign
                desired_angular_speed = self.cmd[1]  # TODO check sign
#                print((x, y, heading), desired_speed, desired_angular_speed, self.cmd)
                self.send_speed_cmd(desired_speed, desired_angular_speed)
            elif channel == 'emergency_stop':
                if self.raise_exception_on_stop and data:
                    raise EmergencyStopException()
            for monitor_update in self.monitors:
                monitor_update(self)

    def send_speed_cmd(self, speed, angular_speed):
        return self.bus.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    def set_speed(self, desired_speed, desired_wheel_heading):
        # TODO split this for Car and Spider mode
        speed = int(round(desired_speed))
#        desired_steering = int(-512 * math.degrees(desired_wheel_heading) / 360.0)
        desired_steering = desired_wheel_heading  # HACK

        if speed != 0:
            if self.wheel_heading is None:
                speed = 1  # in in place
            else:
                 d = math.degrees(normalizeAnglePIPI(self.wheel_heading - desired_wheel_heading))
                 if abs(d) > 20.0:
                     speed = 1  # turn in place (II.)

        self.cmd = [speed, desired_steering]

    def start(self):
        pass

    def request_stop(self):
        self.bus.shutdown()

    def join(self):
        pass

    def register(self, callback):
        self.monitors.append(callback)
        return callback

    def unregister(self, callback):
        assert callback in self.monitors
        self.monitors.remove(callback)

    def wait(self, dt):
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()

    def play0(self):
        self.wait(timedelta(seconds=1))
        self.set_speed(1, math.radians(50))
        self.wait(timedelta(seconds=5))
        self.set_speed(10, 0)
        self.wait(timedelta(seconds=5))
        self.set_speed(0, 0)
        self.wait(timedelta(seconds=1))

    def play(self):
        try:
            with EmergencyStopMonitor(self):

                print("Waiting for valid GPS position...")
                while self.last_gps_position is None or self.last_gps_position == INVALID_COORDINATES:
                    self.update()
                print(self.last_gps_position)

                print("Wait for valid IMU...")
                while self.last_imu_yaw is None:
                    self.update()
                print(self.last_imu_yaw)

                print("Wait for steering info...")
                while self.wheel_heading is None:
                    self.update()
                print(math.degrees(self.wheel_heading))

                print("Ready", self.goals)
                for goal in self.goals:
                    print("Goal at %.2fm" % geo_length(self.last_gps_position, goal))
                    angle = geo_angle(self.last_gps_position, goal)
                    if angle is not None:
                        print("Heading %.1fdeg, imu" % math.degrees(angle), self.last_imu_yaw)
                    else:
                        print("Heading None, imu", self.last_imu_yaw)
                    self.navigate_to_goal(goal, timedelta(seconds=200))
        except EmergencyStopException:
            print("EMERGENCY STOP (wait 3s)")
            self.set_speed(0, 0)
            start_time = self.time
            while self.time - start_time < timedelta(seconds=3):
                self.set_speed(0, 0)
                self.update()

    def navigate_to_goal(self, goal, timeout):
        start_time = self.time
        self.last_gps_position_angle = self.last_gps_position
        gps_angle = None
        while geo_length(self.last_gps_position, goal) > 1.0 and self.time - start_time < timeout:
            desired_heading = normalizeAnglePIPI(geo_angle(self.last_gps_position, goal))
            desired_heading = 0   # HACK!!!
            step = geo_length(self.last_gps_position, self.last_gps_position_angle)
            if False:  #step > 1.0:
                gps_angle = normalizeAnglePIPI(geo_angle(self.last_gps_position_angle, self.last_gps_position))
                print('step', step, math.degrees(gps_angle))
                self.last_gps_position_angle = self.last_gps_position
                desired_wheel_heading = normalizeAnglePIPI(desired_heading - gps_angle + self.wheel_heading)

            if gps_angle is None or self.wheel_heading is None:
                spider_heading = normalizeAnglePIPI(math.radians(180 - self.last_imu_yaw - 35.5))
                desired_wheel_heading = normalizeAnglePIPI(desired_heading-spider_heading)

            self.set_speed(self.maxspeed, desired_wheel_heading)

            prev_time = self.time
            self.update()

            if int(prev_time.total_seconds()) != int(self.time.total_seconds()):
                print(self.time, geo_length(self.last_gps_position, goal), self.last_imu_yaw, self.wheel_heading)

        print("STOP (3s)")
        self.set_speed(0, 0)
        start_time = self.time
        while self.time - start_time < timedelta(seconds=3):
            self.set_speed(0, 0)
            self.update()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='GPS Waypoint Navigation')
    subparsers = parser.add_subparsers(help='sub-command help', dest='command')
    subparsers.required = True
    parser_run = subparsers.add_parser('run', help='run on real HW')
    parser_run.add_argument('config', nargs='+', help='configuration file')
    parser_run.add_argument('--note', help='add description')

    parser_replay = subparsers.add_parser('replay', help='replay from logfile')
    parser_replay.add_argument('logfile', help='recorded log file')
    parser_replay.add_argument('--force', '-F', dest='force', action='store_true', help='force replay even for failing output asserts')
    parser_replay.add_argument('--config', nargs='+', help='force alternative configuration file')
    args = parser.parse_args()

    if args.command == 'replay':
        from replay import replay
        args.module = 'app'
        game = replay(args, application=GPSNav)
        game.play()

    elif args.command == 'run':
        log = LogWriter(prefix='gpsnav-', note=str(sys.argv))
        config = config_load(*args.config)
        log.write(0, bytes(str(config), 'ascii'))  # write configuration
        robot = Recorder(config=config['robot'], logger=log, application=GPSNav)
        game = robot.modules['app']  # TODO nicer reference
        robot.start()
        game.play()
        recorder.finish()
    else:
        assert False, args.command  # unsupported command

# vim: expandtab sw=4 ts=4
