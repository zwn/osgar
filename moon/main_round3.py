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

CAMERA_FOCAL_LENGTH = 381
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

CAMERA_ANGLE_DRIVING = 0.1
CAMERA_ANGLE_LOOKING = 0.5
CAMERA_ANGLE_CUBESAT = 0.78
CAMERA_ANGLE_HOMEBASE = 0.3

class ChangeDriverException(Exception):
    pass


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

class LidarCollisionException(Exception):
    pass


class LidarCollisionMonitor:
    def __init__(self, robot):
        self.robot = robot

    def update(self, robot, channel):
        if channel == 'scan':
            size = len(robot.scan)
            # measure distance only in 160 degree angle
            # robot is ~2.21m wide (~1.2m x 2 with wiggle room), with 160 angle need to have 1.2m clearance (x = 1.2 / sin(160/2))
            if min_dist(robot.scan[55:215]) < 1.2 and not robot.inException:
                raise LidarCollisionException()

    # context manager functions
    def __enter__(self):
        self.callback = self.robot.register(self.update)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.callback)


class SpaceRoboticsChallenge(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("desired_speed", "pose2d", "pose3d", "request_origin", "driving_recovery", "artf_cmd", "set_cam_angle", "set_brakes", "follow_object")
        self.monitors = []
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
        self.yaw_offset = 0.0
        self.yaw, self.pitch, self.roll = 0, 0, 0
        self.xyz = (0, 0, 0)  # 3D position for mapping artifacts
        self.xyz_quat = [0, 0, 0]
        self.offset = (0, 0, 0)
        self.min_front_distance = 0.0
        
        self.score = 0
        self.current_driver = None
        self.cubesat_location = None
        self.camera_angle = CAMERA_ANGLE_DRIVING
        self.camera_change_triggered_time = None
        self.brakes_on = False
        
        self.origin_updated = False

        self.homebase_reached = False
        self.homebase_reported = False

        self.cubesat_reached = False
        self.cubesat_reported = False
        
        self.inException = False
        
        self.last_status_timestamp = None
        
        self.virtual_bumper = None
        self.rand = Random(0)

    def register(self, callback):
        self.monitors.append(callback)
        return callback

    def unregister(self, callback):
        assert callback in self.monitors
        self.monitors.remove(callback)
        
    def send_speed_cmd(self, speed, angular_speed):
        if self.virtual_bumper is not None:
            self.virtual_bumper.update_desired_speed(speed, angular_speed)
        self.bus.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    def set_cam_angle(self, angle):
        self.camera_angle = angle
        print ("Set camera angle to: %f" % angle)
        self.camera_change_triggered_time = self.time
        self.publish('set_cam_angle', bytes('set_cam_angle %f\n' % angle, encoding='ascii'))

    def set_brakes(self, on):
        self.brakes_on = on
        print ("Setting brakes to: %s" % on)
        self.publish('set_brakes', bytes('set_brakes %s\n' % ('on' if on else 'off'), encoding='ascii'))
            
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
                self.bus.publish('driving_recovery', True)
                raise VirtualBumperException()

    def on_driving_control(self, timestamp, data):
        # someone else took over driving
        self.current_driver = data
        if data is None:
            self.set_cam_angle(CAMERA_ANGLE_DRIVING)
            print("Driving returned to main")
        else:
            print("Current driver: %s" % self.current_driver)
            if self.current_driver == "cubesat":
                self.set_cam_angle(CAMERA_ANGLE_LOOKING)
            elif self.current_driver == "homebase":
                self.set_cam_angle(CAMERA_ANGLE_DRIVING)
            else:
                self.set_cam_angle(CAMERA_ANGLE_DRIVING)
        if not self.inException: # do not interrupt driving if processing an exception
            raise ChangeDriverException(data)

    def on_object_reached(self, timestamp, data):
        object_type = data
        if (object_type == "cubesat"):
            self.current_driver = "cubesat-finish"
            self.set_brakes(True)
            self.set_cam_angle(CAMERA_ANGLE_CUBESAT)
            self.cubesat_reached = True
            self.publish('request_origin', True) # response to this is required, if none, rover will be stopped forever
        elif (object_type == "homebase"): # upon handover, robot should be moving straight
            if self.cubesat_reported:
                self.set_cam_angle(CAMERA_ANGLE_HOMEBASE) # look somewhat up in case we are approaching on a downhill slope
                self.current_driver = "homebase-finish"
                self.homebase_reached = True
            else:
                print("Reached reportable home base destination, need to find cubesat first though")
                
    def on_score(self, timestamp, data):
        self.score = data[0]

    def interpolate_distance(self, pixels):
        # linearly interpolate in between measured values (pixels, distance)
        # line from 2 points: https://www.desmos.com/calculator/md6buy4efz
        # plot 2D points: https://www.desmos.com/calculator/mhq4hsncnh
        # plot 3D points: https://technology.cpm.org/general/3dgraph/
        
        observed_values = [(27.5, 24.5), (29.5, 20.5), (41,18.3), (45,15.5), (51, 15.1), (60, 13.5), (62, 11.9)]

        t1 = None
        
        if pixels < observed_values[0][0]:
            return 25 # catch-all for furthest distances outside measured range
        else:
            for t2 in observed_values:
                if pixels < t2[0]:
                    x2 = t2[0]
                    y2 = t2[1]
                    x1 = t1[0]
                    y1 = t1[1]
                    m = (y2 - y1) / (x2 - x1)
                    return m * (pixels - x1) + y1
                else:
                    t1 = t2

        i = len(observed_values) - 2
        x2 = t2[i]
        y2 = t2[i+1]
        x1 = t1[i]
        y1 = t1[i+1]
        m = (y2 - y1) / (x2 - x1)
        return m * (pixels - x1) + y1
                    
        
    def on_artf(self, timestamp, data):
        artifact_type, img_x, img_y, img_w, img_h = data

        if self.cubesat_reached and not self.origin_updated:
            self.bus.publish('request_origin', True) # keep requesting origin TODO: get rid of this
        
        if self.cubesat_reached and self.time - self.camera_change_triggered_time > timedelta(seconds=3) and artifact_type == "cubesat" and self.origin_updated and not self.cubesat_reported:
            print(self.time, "app: Final frame x=%d y=%d w=%d h=%d" % (data[1], data[2], data[3], data[4]))
            angle_x = math.atan( (CAMERA_WIDTH / 2 - (img_x + img_w/2) ) / float(CAMERA_FOCAL_LENGTH))
            angle_y = math.atan( (CAMERA_HEIGHT / 2 - (img_y + img_h/2) ) / float(CAMERA_FOCAL_LENGTH))

            distance = self.interpolate_distance((img_w + img_h) / 2)
            ax = self.nasa_yaw + angle_x
            ay = self.nasa_pitch + angle_y + self.camera_angle - self.nasa_pitch # direction of camera; NOTE: gimbal changes the actual angle dynamically so pitch should be offset
            x, y, z = self.nasa_xyz
            print("Using pose: xyz=[%f %f %f] orientation=[%f %f %f]" % (x, y, z, self.nasa_roll, self.nasa_pitch, self.nasa_yaw))
            print("In combination with view angle %f %f and distance %f" % (ax, ay, distance))
            ox = math.cos(ax) * math.cos(ay) * distance
            oy = math.sin(ax) * math.cos(ay) * distance
            oz = math.sin(ay) * distance
            self.cubesat_location = (x+ox, y+oy, z+oz)
            print (self.time, "app: Object offset calculated at: [%f %f %f]" % (ox, oy, oz))
            print (self.time, "app: Reporting estimated object location at: [%f,%f,%f]" % (x+ox, y+oy, z+oz))

            s = '%s %.2f %.2f %.2f\n' % (artifact_type, x+ox, y+oy, z+oz)
            self.publish('artf_cmd', bytes('artf ' + s, encoding='ascii'))

            self.cubesat_reported = True
            self.set_brakes(False)
            # time to start looking for homebase
            self.bus.publish('follow_object', ['homebase'])
            self.current_driver = None
            if not self.inException:
                raise ChangeDriverException(None) # interrupt main wait, give driving back
            
        
    def update(self):

        # print status periodically - location
        if self.time is not None:
            if self.last_status_timestamp is None:
                self.last_status_timestamp = self.time
            elif self.time - self.last_status_timestamp > timedelta(seconds=8):
                self.last_status_timestamp = self.time
                x, y, z = self.xyz
                print (self.time, "Loc: [%f %f %f] [%f %f %f]; Driver: %s; Score: %d" % (x, y, z, self.roll, self.pitch, self.yaw, self.current_driver, self.score))

                # keep sending TODO: get rid of this
                self.set_brakes(self.brakes_on)
                if self.cubesat_location is not None:
                    s = '%s %.2f %.2f %.2f\n' % ('cubesat', self.cubesat_location[0], self.cubesat_location[1], self.cubesat_location[2], )
                    self.publish('artf_cmd', bytes('artf ' + s, encoding='ascii'))
                if self.homebase_reported:
                    self.publish('artf_cmd', bytes('artf homebase\n', encoding='ascii'))


        channel = super().update()
#        handler = getattr(self, "on_" + channel, None)
#        if handler is not None:
#            handler(self.time, data)
        if channel == 'pose2d':
            self.on_pose2d(self.time, self.pose2d)
        elif channel == 'score':
            self.on_score(self.time, self.score)
        elif channel == 'artf':
            self.on_artf(self.time, self.artf)
        elif channel == 'object_reached':
            self.on_object_reached(self.time, self.object_reached)
        elif channel == 'driving_control':
            self.on_driving_control(self.time, self.driving_control)
        elif channel == 'scan':
            self.local_planner.update(self.scan)
            self.min_front_distance = min_dist(self.scan[len(self.scan)//3:2*len(self.scan)//3])
            if self.min_front_distance < 5 and not self.homebase_reported and self.homebase_reached:
                print ("app: Reporting homebase reached to server, distance %f: " % self.min_front_distance)
                self.publish('artf_cmd', bytes('artf homebase\n', encoding='ascii'))
                self.homebase_reported = True
                self.set_brakes(True)
                #self.current_driver = None
                #raise ChangeDriverException(None)
                # THIS IS THE END OF THE RUN FOR NOW
                
        elif channel == 'origin':
            data = self.origin[:]  # the same name is used for message as internal data
            self.xyz = data[1:4]
            
            qx, qy, qz, qw = data[4:]
            self.roll = qx
            self.pitch = qy
            self.yaw = qz

            self.bus.publish('follow_object', ['homebase']) # stop looking for cubesat
            print(self.time, "Origin received, internal position updated")
            # robot should be stopped right now (using brakes once available)
            # lift camera to max, object should be (back) in view
            # trigger recognition, get bounding box and calculate fresh angles

            # TODO: this separate storage of reported numbers is temporary, need OSGAR to accept true values and update its own data
            self.nasa_xyz = self.xyz
            sinr_cosp = 2 * (qw * qx + qy * qz);
            cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
            self.nasa_roll = math.atan2(sinr_cosp, cosr_cosp);

            sinp = 2 * (qw * qy - qz * qx);
            if abs(sinp) >= 1:
                self.nasa_pitch = math.copysign(math.pi / 2, sinp);
            else:
                self.nasa_pitch = math.asin(sinp);

            siny_cosp = 2 * (qw * qz + qx * qy);
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
            self.nasa_yaw = math.atan2(siny_cosp, cosy_cosp);
            print (self.time, "True pose received: xyz=[%f,%f,%f], roll=%f, pitch=%f, yaw=%f" % (data[1],data[2],data[3],self.nasa_roll, self.nasa_pitch, self.nasa_yaw))
            # tilt camera all the way up, it should trigger a detection again
            self.origin_updated = True
            
        elif channel == 'rot':
            temp_yaw, self.pitch, self.roll = [normalizeAnglePIPI(math.radians(x/100)) for x in self.rot]
            if self.yaw_offset is None:
                self.yaw_offset = -temp_yaw
            self.yaw = temp_yaw + self.yaw_offset

            # maintain camera level
            cam_angle = self.camera_angle - self.pitch 
            self.publish('set_cam_angle', bytes('set_cam_angle %f\n' % cam_angle, encoding='ascii'))
            
            if not self.inException and self.pitch > 0.6:
                # TODO pitch can also go the other way if we back into an obstacle
                # TODO: robot can also roll if it runs on a side of a rock while already on a slope
                self.bus.publish('driving_recovery', True)
                print (self.time, "app: Excess pitch or roll, going back")
                raise VirtualBumperException()

        for m in self.monitors:
            m(self, channel)
            
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

    def try_step_around(self):
        #TODO: possibly move sideways instead of two 90 degree turns
        self.turn(math.radians(90), timeout=timedelta(seconds=10))

        # recovered enough at this point to switch to another driver (in case you see cubesat while doing the 3m drive or the final turn)
        self.bus.publish('driving_recovery', False)
        
        self.go_straight(3.0, timeout=timedelta(seconds=20))
        self.turn(math.radians(-90), timeout=timedelta(seconds=10))
        
    def run(self):
        try:
            print('Wait for definition of last_position and yaw')
            while self.last_position is None or self.yaw is None:
                self.update()  # define self.time
            print('done at', self.time)

            self.bus.publish('follow_object', ['cubesat', 'homebase'])

            
            try:
                self.set_cam_angle(CAMERA_ANGLE_LOOKING)
                self.turn(math.radians(360), timeout=timedelta(seconds=20)) # turn bumper needs to be virtually disabled as turns happen in place and bumper measures position change
            except ChangeDriverException as e:
                print(self.time, "Initial turn interrupted by driver: %s" % e)
            except VirtualBumperException:
                self.inException = True
                self.set_cam_angle(CAMERA_ANGLE_DRIVING)
                print(self.time, "Initial Turn Virtual Bumper!")
                # TODO: if detector takes over driving within initial turn, rover may be actually going straight at this moment
                # also, it may be simple timeout, not a crash
                self.virtual_bumper = None
                deg_angle = self.rand.randrange(90, 180)
                deg_sign = self.rand.randint(0,1)
                if deg_sign:
                    deg_angle = -deg_angle
                self.turn(math.radians(deg_angle), timeout=timedelta(seconds=10))
                self.inException = False
                self.bus.publish('driving_recovery', False)
                
            last_walk_start = 0.0
            start_time = self.time
            while self.time - start_time < timedelta(minutes=40):
                additional_turn = 0
                last_walk_start = self.time
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=4), 0.1)
                    with LidarCollisionMonitor(self):
                        if self.current_driver is None and not self.brakes_on:
                            self.set_cam_angle(CAMERA_ANGLE_DRIVING)
                            self.go_straight(100.0, timeout=timedelta(minutes=2))
                        else:
                            self.wait(timedelta(minutes=2)) # allow for self driving, then timeout   
                    self.update()
                except ChangeDriverException as e:
                    continue

                except (VirtualBumperException, LidarCollisionException) as e:
                    self.inException = True
# TODO: crashes if an exception (e.g., excess pitch) occurs while handling an exception (e.g., virtual/lidar bump)
                    print(self.time, repr(e))
                    last_walk_end = self.time
                    self.virtual_bumper = None
                    self.set_cam_angle(CAMERA_ANGLE_DRIVING)
                    self.go_straight(-2.0, timeout=timedelta(seconds=10)) # this should be reasonably safe, we just came from there
                    if last_walk_end - last_walk_start > timedelta(seconds=20): # if we went more than 20 secs, try to continue a step to the left
                        # TODO: this is not necessarily safe, need to protect against pitch, etc again
                        self.try_step_around()
                    else:
                        self.bus.publish('driving_recovery', False)

                    self.inException = False

                    print ("Time elapsed since start of previous leg: %d sec" % (last_walk_end.total_seconds()-last_walk_start.total_seconds()))
                    if last_walk_end - last_walk_start > timedelta(seconds=40):
                        # if last step only ran short time before bumper (this includes bumper timeout time), time for a large random turn
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
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=20), 0.1)

                    # when looking for satellite, do 360 each turn and look up
                    x,y,z = self.xyz
                    if z > 0: # no need to look around if in crater
                        self.set_cam_angle(CAMERA_ANGLE_LOOKING)
                        self.turn(math.radians(360), timeout=timedelta(seconds=20))
                    
                    self.turn(math.radians(deg_angle), timeout=timedelta(seconds=30))
                except ChangeDriverException as e:
                    continue
                    
                except VirtualBumperException:
                    self.inException = True
                    self.set_cam_angle(CAMERA_ANGLE_DRIVING)
                    print(self.time, "Turn Virtual Bumper!")
                    self.virtual_bumper = None
                    if self.current_driver is not None:
                        # probably didn't throw in previous turn but during self driving
                        self.go_straight(-2.0, timeout=timedelta(seconds=10))
                        self.try_step_around()
                    self.turn(math.radians(-deg_angle), timeout=timedelta(seconds=30))
                    self.inException = False
                    self.bus.publish('driving_recovery', False)
        except BusShutdownException:
            pass


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Space Robotics Challenge 2')
    args = parser.parse_args()

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
