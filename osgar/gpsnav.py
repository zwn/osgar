"""
  GPS Navigation for RoboOrienteering 2019
"""
# based on Spide3 "ro2018.py"

import argparse
import sys
import math
from datetime import timedelta
from queue import Queue

from osgar.lib.mathex import normalizeAnglePIPI
from osgar.node import Node
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
        if (robot.status & RoboOrienteering2018.EMERGENCY_STOP) == 0:
            raise EmergencyStopException()

    # context manager functions
    def __enter__(self):
        self.callback = self.robot.register(self.update)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.callback)


class GPSNavigation(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.maxspeed = config.get('maxspeed', 0.5)
        self.goals = [latlon2xy(lat, lon) for lat, lon in config['waypoints']]
        self.last_position = None  # (lon, lat) in milliseconds
        self.last_imu_yaw = None  # magnetic north in degrees
        self.cmd = [0, 0]
        self.last_position_angle = None  # for angle computation from dGPS
        self.monitors = []

    def send_speed_cmd(self, speed, angular_speed):
        return self.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    def update(self):
        channel = super().update()  # define self.time
        if channel == 'position':
            self.last_position = self.position
        elif channel == 'rot':  # should be rather 'rotation'
            yaw, pitch, roll = self.rot
            self.last_imu_yaw = math.radians(yaw/100.0)

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

    def run0(self):
        self.wait(timedelta(seconds=1))
        self.send_speed_cmd(0.5, math.radians(20))
        self.wait(timedelta(seconds=5))
        self.send_speed_cmd(0.5, 0)
        self.wait(timedelta(seconds=5))
        self.send_speed_cmd(0, 0)
        self.wait(timedelta(seconds=1))

    def run1(self):
        from osgar.lib.horizontal_lidar_ocgm import HorizontalLidarOcgm, OCGM_CELLS_COUNT, OCGM_CELLS_PER_METER
        from osgar.lib.vfh import VFH
        from osgar.lib.visual_log import VisualLog

        STEER_FACTOR = 1 #0.5

        visualLog = VisualLog()
        visualLog.init()
        horizontalLidarOcgm = HorizontalLidarOcgm(cells=OCGM_CELLS_COUNT/2,
                                               res=1/OCGM_CELLS_PER_METER,
                                               p_d=0.9,
                                               p_nd=0.4,
                                               decay=0.98,
                                               pose = [0,0,0],
                                               cols = 190,
                                               rows = int(OCGM_CELLS_COUNT/2)+1)
        vfh = VFH(horizontalLidarOcgm,visualLog)

        self.wait(timedelta(seconds=1))

        globalWaypoint = (100, 0)
        while True:
            channel = self.update()
            timestamp = self.time
            scan = self.scan  # [1000]*270
            x, y, heading = self.pose2d
            pose = (x/1000.0, y/1000.0, math.radians(heading/100.0))

            visualLog.init()
            #OCGM update
            localMap = horizontalLidarOcgm.update(scan, timestamp, pose)
            horizontalLidarOcgm.drawImageToVisualLog(visualLog, pose)

            #local planner update
            desired_direction = vfh.update(globalWaypoint, pose, localMap)
            if desired_direction is None:
                print("No direction!")
                self.send_speed_cmd(0, 0)
            else:
                desired_angular_speed = normalizeAnglePIPI(math.pi - desired_direction) * STEER_FACTOR
                print(self.time, desired_direction, desired_angular_speed)
            visualLog.drawCar(pose)
            visualLog.show()

        self.wait(timedelta(seconds=1))

    def run(self):
        print("Waiting for valid GPS position...")
        while self.last_position is None or self.last_position == INVALID_COORDINATES:
            self.update()
        print(self.last_position)

        print("Wait for valid IMU...")
        while self.last_imu_yaw is None:
            self.update()
        print(self.last_imu_yaw)

        print("Ready", self.goals)
        try:
            with EmergencyStopMonitor(self):
                for goal in self.goals:
                    print("Goal at %.2fm" % geo_length(self.last_position, goal))
                    angle = geo_angle(self.last_position, goal)
                    if angle is not None:
                        print("Heading %.1fdeg, imu" % math.degrees(angle), self.last_imu_yaw)
                    else:
                        print("Heading None, imu", self.last_imu_yaw)
                    self.navigate_to_goal(goal, timedelta(seconds=600))
        except EmergencyStopException:
            print("EMERGENCY STOP (wait 3s)")
            self.send_speed_cmd(0, 0)
            start_time = self.time
            while self.time - start_time < timedelta(seconds=3):
                self.update()

    def navigate_to_goal(self, goal, timeout):
        start_time = self.time
        self.last_position_angle = self.last_position
        gps_angle = None
        while geo_length(self.last_position, goal) > 1.0 and self.time - start_time < timeout:
            desired_heading = normalizeAnglePIPI(geo_angle(self.last_position, goal))
            step = geo_length(self.last_position, self.last_position_angle)
            if step > 1.0:
                gps_angle = normalizeAnglePIPI(geo_angle(self.last_position_angle, self.last_position))
                print('step', step, math.degrees(gps_angle))
                self.last_position_angle = self.last_position
                desired_wheel_heading = normalizeAnglePIPI(desired_heading - gps_angle)

            if gps_angle is None:
#                spider_heading = normalizeAnglePIPI(math.radians(180 - self.last_imu_yaw - 35.5))
                spider_heading = normalizeAnglePIPI(math.radians(180 - self.last_imu_yaw))
                desired_wheel_heading = normalizeAnglePIPI(desired_heading-spider_heading)

            self.send_speed_cmd(self.maxspeed, desired_wheel_heading)

            prev_time = self.time
            self.update()

            if int(prev_time.total_seconds()) != int(self.time.total_seconds()):
                print(self.time, geo_length(self.last_position, goal), self.last_imu_yaw)

        print("STOP (3s)")
        self.send_speed_cmd(0, 0)
        start_time = self.time
        while self.time - start_time < timedelta(seconds=3):
            self.update()


if __name__ == "__main__":
    from osgar.launcher import launch

    launch(app=GPSNavigation, description='GPS Navigation', prefix='gpsnav-')

# vim: expandtab sw=4 ts=4
