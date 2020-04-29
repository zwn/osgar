"""
  Space Robotics Challenge 2
"""

import math
from random import Random
from datetime import timedelta

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.lib import quaternion
from osgar.lib.mathex import normalizeAnglePIPI
from osgar.lib.virtual_bumper import VirtualBumper

from subt.local_planner import LocalPlanner


class VirtualBumperException(Exception):
    pass


def distance(pose1, pose2):
    return math.hypot(pose1[0] - pose2[0], pose1[1] - pose2[1])


def min_dist(laser_data):
    if len(laser_data) > 0:
        # remove ultra near reflections and unlimited values == 0
        laser_data = [x if x > 10 else 10000 for x in laser_data]
        return min(laser_data)/1000.0
    return 0


class SpaceRoboticsChallenge(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("desired_speed", "artf_xyz", "artf_cmd", "pose2d", "pose3d", "request_origin", "bucket_cmd", "driving_recovery")
        self.last_position = None
        self.max_speed = 1.0  # oficial max speed is 1.5m/s
        self.max_angular_speed = math.radians(60)

        self.min_safe_dist = config.get('min_safe_dist', 2.0)
        self.dangerous_dist = config.get('dangerous_dist', 1.5)
        self.safety_turning_coeff = config.get('safety_turning_coeff', 0.8)
        scan_subsample = config.get('scan_subsample', 1)
        obstacle_influence = config.get('obstacle_influence', 0.8)
        direction_adherence = math.radians(config.get('direction_adherence', 90))
        self.local_planner = LocalPlanner(
                obstacle_influence=obstacle_influence,
                direction_adherence=direction_adherence,
                max_obstacle_distance=4.0,
                scan_subsample=scan_subsample,
                max_considered_obstacles=100)

        self.origin = None  # unknown initial position
        self.origin_quat = quaternion.identity()
        self.start_pose = None
        self.yaw_offset = None
        self.yaw, self.pitch, self.roll = 0, 0, 0
        self.xyz = (0, 0, 0)  # 3D position for mapping artifacts
        self.xyz_quat = [0, 0, 0]
        self.offset = (0, 0, 0)
        self.score = 0
        self.bucket_status = None
        self.is_someone_else_driving = False

        # TODO: account for working on an incline
        self.scoop_time = None
        self.scoop_index = 0
        self.bucket_scoop_part = True # True for scoop part, False for drop part
        self.bucket_scoop_sequence = (
            # [<seconds to execute>, [mount, base, distal, bucket]] 
            [12, [-0.6, -0.8, 3.2]], # get above scooping position
            [4, [ 0.4, 1.0, 1.9]], # lower to scooping position
            [2, [ 0.4, 1.0, 3.2]], # scoop volatiles
            [8, [ -0.6, -0.8, 3.9]] # lift up bucket with volatiles
            )
        self.bucket_drop_sequence = (
            [12, [-0.6, -0.8, 3.9]], # turn towards dropping position
            [4, [-0.3, -0.8, 3.9]], # extend arm
            [4, [-0.3, -0.8, 0]], # drop
            [4, [-0.6, -0.8, 3.2]] # back to neutral/travel position
        )
        
        self.last_artf = None
        
        self.inException = False
        
        self.last_volatile_distance = None
        self.last_vol_index = None

        self.last_status_timestamp = None
        
        self.virtual_bumper = None
        self.rand = Random(0)

    def send_speed_cmd(self, speed, angular_speed):
        if self.virtual_bumper is not None:
            self.virtual_bumper.update_desired_speed(speed, angular_speed)
        self.bus.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    def send_bucket_position(self, bucket_params):
        mount, basearm, distalarm, bucket = bucket_params
        s = '%f %f %f %f\n' % (mount, basearm, distalarm, bucket)
        self.publish('bucket_cmd', bytes('bucket_position ' + s, encoding='ascii'))
        
    def on_pose2d(self, timestamp, data):
        x, y, heading = data
        pose = (x / 1000.0, y / 1000.0, math.radians(heading / 100.0))
        if self.last_position is not None:
            dist = math.hypot(pose[0] - self.last_position[0], pose[1] - self.last_position[1])
            direction = ((pose[0] - self.last_position[0]) * math.cos(self.last_position[2]) +
                         (pose[1] - self.last_position[1]) * math.sin(self.last_position[2]))
            if direction < 0:
                dist = -dist
        else:
            dist = 0.0
        self.last_position = pose
        if self.start_pose is None:
            self.start_pose = pose
        x, y, z = self.xyz
        x += math.cos(self.pitch) * math.cos(self.yaw) * dist
        y += math.cos(self.pitch) * math.sin(self.yaw) * dist
        z += math.sin(self.pitch) * dist
        x0, y0, z0 = self.offset
        self.last_send_time = self.bus.publish('pose2d', [round((x + x0) * 1000), round((y + y0) * 1000),
                                    round(math.degrees(self.yaw) * 100)])
        self.xyz = x, y, z
        if self.virtual_bumper is not None:
            self.virtual_bumper.update_pose(self.time, pose)
            if not self.inException and self.virtual_bumper.collision():
                self.inException = True
                self.bus.publish('driving_recovery', True)
                raise VirtualBumperException()

    def on_driving_control(self, timestamp, data):
        # true if someone else took over driving
        self.is_someone_else_driving = data
        print("Someone else is driving %r" % data)

    def on_object_reached(self, timestamp, data):
        object_type, angle_x, angle_y, distance = data
        if (object_type == "cubesat"):
            self.bus.publish('request_origin', True)
            print ("After origin received, this would be followed by cubesat location report")

        
    def on_artf(self, timestamp, data):
        # called by incoming volatile sensor report (among other sources)
        # 0 vol_type, 1 distance_to, 2 vol_index
        artifact_type = data[0]  # meters ... TODO distinguish CubeSat, volatiles, ProcessingPlant

        if self.last_artf is None:
# disable for round 3            self.bus.publish('request_origin', True)
            self.last_artf = artifact_type
        
        distance_to = data[1]
        vol_index = data[2]

        if self.last_volatile_distance is None:
            self.last_volatile_distance = distance_to
        elif self.last_volatile_distance > distance_to:
#            print ("Volatile detection %d, getting closer: %f" % (vol_index, distance_to))
            self.last_volatile_distance = distance_to
        elif self.last_vol_index is None or vol_index != self.last_vol_index:
            self.last_vol_index = vol_index
            self.last_volatile_distance = None
            # TODO: this must be adjusted to report the position of the sensor, not the robot (which NASA will update their code for at some point)
            # the best known distance was in reference to mutual position of the sensor and the volatile
            ax, ay, az = self.xyz
#            print ("Volatile detection, starting to go further, reporting %f %f" % (ax, ay))

            # TODO (maybe): if not accepted, try again?
            s = '%s %.2f %.2f %.2f\n' % (artifact_type, ax, ay, 0.0)
            self.publish('artf_cmd', bytes('artf ' + s, encoding='ascii'))
        else:
            self.last_volatile_distance = None
#            print ("Previously visited volatile %d, not reporting" % vol_index)
            

    def on_score(self, timestamp, data):
        self.score = data[0]

    def on_bucket_info(self, timestamp, data):
        self.bucket_status = data
        
    def update(self):

        # demo scooping and dropping material with the excavator
        if self.time is not None:
            if self.scoop_time is None or self.time > self.scoop_time:
                if self.bucket_scoop_part:
                    duration, bucket_params = self.bucket_scoop_sequence[self.scoop_index]
                    target_angle = 3.4 # demo scooping towards the back of the vehicle
                else:
                    duration, bucket_params = self.bucket_drop_sequence[self.scoop_index]
                    target_angle = 0.2 # demo dropping towards the rear of the vehicle
#                print ("bucket_position %f %f %f " % (bucket_params[0], bucket_params[1],bucket_params[2]))
                self.send_bucket_position([target_angle, *bucket_params])
                self.scoop_time = self.time + timedelta(seconds=duration)
                if self.scoop_index + 1 == len(self.bucket_scoop_sequence if self.bucket_scoop_part else self.bucket_drop_sequence):
                    self.scoop_index = 0
                    self.bucket_scoop_part = not self.bucket_scoop_part
                else:
                    self.scoop_index += 1
                    
                

        # print status periodically - location and content of bucket if any
        if self.time is not None:
            if self.last_status_timestamp is None:
                self.last_status_timestamp = self.time
            elif self.time - self.last_status_timestamp > timedelta(seconds=8):
                self.last_status_timestamp = self.time
                x, y, z = self.xyz
                print ("Loc: %f %f %f; Score: %d" % (x, y, z, self.score))
                if self.bucket_status is not None and self.bucket_status[1] > 0:
                    print ("Bucket content: Type: %s idx: %d mass: %f" % (self.bucket_status[0], self.bucket_status[1], self.bucket_status[2]))
                

        
        channel = super().update()
#        handler = getattr(self, "on_" + channel, None)
#        if handler is not None:
#            handler(self.time, data)
        if channel == 'pose2d':
            self.on_pose2d(self.time, self.pose2d)
        elif channel == 'artf':
            self.on_artf(self.time, self.artf)
        elif channel == 'score':
            self.on_score(self.time, self.score)
        elif channel == 'object_reached':
            self.on_object_reached(self.time, self.object_reached)
        elif channel == 'driving_control':
            self.on_driving_control(self.time, self.driving_control)
        elif channel == 'bucket_info':
            self.on_bucket_info(self.time, self.bucket_info)
        elif channel == 'scan':
            self.local_planner.update(self.scan)
        elif channel == 'origin':
            data = self.origin[:]  # the same name is used for message as internal data
            self.xyz = data[1:4]
            
            qx, qy, qz, qw = data[4:]
            self.roll = qx
            self.pitch = qy
            self.yaw = qz

            print("Origin received, internal position updated")
            # report location as fake artifact
            artifact_data = self.last_artf
            ax, ay, az = self.xyz
            self.bus.publish('artf_xyz', [[artifact_data, round(ax*1000), round(ay*1000), round(az*1000)]])

        elif channel == 'rot':
            temp_yaw, self.pitch, self.roll = [normalizeAnglePIPI(math.radians(x/100)) for x in self.rot]
            if self.yaw_offset is None:
                self.yaw_offset = -temp_yaw
            self.yaw = temp_yaw + self.yaw_offset
            if not self.inException and self.pitch > 0.5:
                self.inException = True
                self.bus.publish('driving_recovery', True)
                print ("Excess pitch, going back down")
                raise VirtualBumperException()

        return channel

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
                print("go_straight - timeout at %.1fm" % distance(start_pose, self.last_position))
                break
        self.send_speed_cmd(0.0, 0.0)

    def turn(self, angle, with_stop=True, speed=0.0, timeout=None):
        print(self.time, "turn %.1f" % math.degrees(angle))
        if angle >= 0:
            self.send_speed_cmd(speed, self.max_angular_speed)
        else:
            self.send_speed_cmd(speed, -self.max_angular_speed)
        start_time = self.time
        # problem with accumulated angle

        sum_angle = 0.0
        prev_angle = self.yaw
        while abs(sum_angle) < abs(angle):
            self.update()
            sum_angle += normalizeAnglePIPI(self.yaw - prev_angle)
            prev_angle = self.yaw
            if timeout is not None and self.time - start_time > timeout:
                print(self.time, "turn - timeout at %.1fdeg" % math.degrees(sum_angle))
                break
        if with_stop:
            self.send_speed_cmd(0.0, 0.0)
            start_time = self.time
            while self.time - start_time < timedelta(seconds=2):
                self.update()
            print(self.time, 'stop at', self.time - start_time)

    def wait(self, dt):  # TODO refactor to some common class
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()

    def go_safely(self, desired_direction):
        safety, safe_direction = self.local_planner.recommend(desired_direction)
        desired_angular_speed = 0.9 * safe_direction
        size = len(self.scan)
        dist = min_dist(self.scan[size//3:2*size//3])
#        print(safe_direction, safety, dist)
        if dist < self.min_safe_dist:
            desired_speed = self.max_speed * (dist - self.dangerous_dist) / (self.min_safe_dist - self.dangerous_dist)
        else:
            desired_speed = self.max_speed
        desired_speed = desired_speed * (1.0 - self.safety_turning_coeff * min(self.max_angular_speed, abs(desired_angular_speed)) / self.max_angular_speed)
        self.send_speed_cmd(desired_speed, desired_angular_speed)
        return safety

    def random_walk(self, timeout):
        start_time = self.time
        while self.time - start_time < timeout:
            if self.update() == 'scan':
                self.go_safely(0.0)

        self.send_speed_cmd(0.0, 0.0)

    def run(self):
        try:
            print('Wait for definition of last_position and yaw')
            while self.last_position is None or self.yaw is None:
                self.update()  # define self.time
            print('done at', self.time)

            try:
#                print ("No turn for volatile search")
                self.turn(math.radians(360), timeout=timedelta(seconds=20)) # turn bumper needs to be virtually disabled as turns happen in place and bumper measures position change
            except VirtualBumperException:
                print(self.time, "Initial Turn Virtual Bumper!")
                self.virtual_bumper = None
                deg_angle = self.rand.randrange(90, 180)
                deg_sign = self.rand.randint(0,1)
                if deg_sign:
                    deg_angle = -deg_angle
                self.turn(math.radians(deg_angle), timeout=timedelta(seconds=20))
                self.inException = False
                self.bus.publish('driving_recovery', False)

            last_walk_start = 0.0
            start_time = self.time
            while self.time - start_time < timedelta(minutes=40):
                if not self.is_someone_else_driving:
                    additional_turn = 0
                    try:
                        last_walk_start = self.time
                        self.virtual_bumper = VirtualBumper(timedelta(seconds=4), 0.1)
                        self.go_straight(100.0, timeout=timedelta(minutes=5))
                        self.update()
                    except VirtualBumperException:
                        print(self.time, "Virtual Bumper!")
                        self.virtual_bumper = None
                        self.go_straight(-1.0, timeout=timedelta(seconds=10))
                        if self.time - last_walk_start > timedelta(seconds=10):
                            self.turn(math.radians(90), timeout=timedelta(seconds=10))
                            self.go_straight(3.0, timeout=timedelta(seconds=10))
                            self.turn(math.radians(-90), timeout=timedelta(seconds=10))
                            
                        self.inException = False
                        self.bus.publish('driving_recovery', False)
                        if self.time - last_walk_start > timedelta(seconds=50): # could take 50 secs to try the maneuver
                            # if last step only ran short time before bumper, time for a large random turn
                            # if it ran long time, maybe worth trying going in the same direction
                            continue
                        additional_turn = 30
                        
                    # next random walk direction should be between 30 and 150 degrees
                    # (no need to go straight back or keep going forward)
                    # if part of virtual bumper handling, add 30 degrees to avoid the obstacle more forcefully
                    deg_angle = self.rand.randrange(30 + additional_turn, 150 + additional_turn)
                    deg_sign = self.rand.randint(0,1)
                    if deg_sign:
                        deg_angle = -deg_angle
                    try:
                        self.virtual_bumper = VirtualBumper(timedelta(seconds=10), 0.1)
                        self.turn(math.radians(deg_angle), timeout=timedelta(seconds=30))
                    except VirtualBumperException:
                        print(self.time, "Turn Virtual Bumper!")
                        self.virtual_bumper = None
                        self.turn(math.radians(-deg_angle), timeout=timedelta(seconds=30))
                        self.inException = False
                        self.bus.publish('driving_recovery', False)
                else:
                    self.wait(timedelta(seconds=1))    
            self.wait(timedelta(seconds=10))
        except BusShutdownException:
            pass


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Space Robotics Challenge 2')
    args = parser.parse_args()

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
