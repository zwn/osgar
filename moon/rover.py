"""
  Moon Rover Driver
"""

# source: (limited access)
#   https://gitlab.com/scheducation/srcp2-competitors/-/wikis/Documentation/API/Simulation_API
# Motor Drive Command Topics
#  /name/fl_wheel_controller/command
#  /name/fr_wheel_controller/command
#  /name/bl_wheel_controller/command
#  /name/br_wheel_controller/command

# Steering Arm Control Topics
#  /name/fr_steering_arm_controller/command
#  /name/fl_steering_arm_controller/command
#  /name/bl_steering_arm_controller/command
#  /name/br_steering_arm_controller/command

# Info
# /name/joint_states  sensor_msgs/JointStates
# /name/skid_cmd_vel  geometry_msgs/Twist
# /name/get_true_pose

# Sensors
# /name/laser/scan  sensor_msgs/LaserScan
# /name/camera/<side>/image_raw  sensor_msgs/Image
# /name/imu  sensor_msgs/Imu

# /name/joint_states  sensor_msgs/JointStates
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.

# Sensor Joint Controller
# /name/sensor_controller/command


import math
from statistics import median
from datetime import timedelta

from osgar.lib.mathex import normalizeAnglePIPI
from osgar.node import Node


def min_dist(laser_data):
    if len(laser_data) > 0:
        # remove ultra near reflections and unlimited values == 0
        laser_data = [x if x > 10 else 20000 for x in laser_data] # NASA scanner goes up to 15m of valid measurement
        return min(laser_data)/1000.0
    return 0

def median_dist(laser_data):
    if len(laser_data) > 0:
        # remove ultra near reflections and unlimited values == 0
        laser_data = [x if x > 10 else 20000 for x in laser_data] # NASA scanner goes up to 15m of valid measurement
        return median(laser_data)/1000.0
    return 0

WHEEL_RADIUS = 0.275  # meters
WHEEL_SEPARATION_WIDTH = 1.87325  # meters
WHEEL_SEPARATION_HEIGHT = 1.5748  # meters

WHEEL_NAMES = ['fl', 'fr', 'bl', 'br']

FOUR_WHEEL_DRIVE_PITCH_THRESHOLD = 0.1
CRAB_DRIVE_ROLL_THRESHOLD = 0.4
CRAB_ROLL_ANGLE = 0.78
STEER_TOWARDS_OBJECT_ANGLE = 0.5
SPEED_ON = 10 #triggers movement when not zero, actual value does not matter

# there could a bump on the road or glitch in the filter so give it some time to re-find

HOMEBASE_KEEP_DISTANCE = 3 # maintain this distance from home base while approaching and going around

CAMERA_FOCAL_LENGTH = 381
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

class Rover(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('cmd', 'pose2d', 'object_reached', "driving_control")
        self.desired_linear_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0
        self.joint_name = None  # updated via Node.update()
        self.debug_arr = []
        self.verbose = False
        self.prev_position = None
        self.pose2d = 0, 0, 0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.yaw_offset = None

        self.started_turning_for_basemarker_timestamp = None
        self.basemarker_centered = False
        self.homebase_final_approach = False
        
        self.currently_following_object = {
            'object_type': None,
            'timestamp': None
            }

        self.object_timeouts = { # in miliseconds
            'homebase': timedelta(seconds=5),
            'cubesat': timedelta(seconds=5),
            'basemarker': timedelta(milliseconds=200)
            }

        
        self.last_artefact_time = None
        self.last_tracked_artefact = None
        
        self.in_driving_recovery = False
        self.objects_to_follow = []

    def on_desired_speed(self, data):
        self.desired_linear_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)

    def on_driving_recovery(self, data):
        self.in_driving_recovery = data
        print (self.time, "Driving recovery changed to: %r" % data)

    def on_follow_object(self, data):
        self.objects_to_follow = data
        print (self.time, "Starting to look for " + ','.join(data))

    def object_reached(self, object_type):
        self.currently_following_object['object_type'] = None
        self.currently_following_object['timestamp'] = None

        self.objects_to_follow.remove(object_type)
        print (self.time, "rover: Reached and no longer looking for %s, reporting to main" % object_type)
        self.bus.publish('object_reached', object_type)
            
        
    # used to follow objects (cubesat, processing plant, other robots, etc)
    def on_artf(self, data):
        # vol_type, x, y, w, h
        # coordinates are pixels of bounding box
        artifact_type = data[0]  # meters ... TODO distinguish CubeSat, volatiles, ProcessingPlant

        center_x = data[1] + data[3] / 2
        center_y = data[2] + data[4] / 2
        bbox_size = (data[3] + data[4]) / 2 # calculate avegage in case of substantially non square matches

        # TODO if detection during turning on the spot, instead of driving straight steering a little, turn back to the direction where the detection happened first
        
        if not self.in_driving_recovery and self.objects_to_follow and artifact_type in self.objects_to_follow: # if in exception, let the exception handling take its course
            if self.currently_following_object['object_type'] is None:
                self.currently_following_object['object_type'] = artifact_type
                self.currently_following_object['timestamp'] = self.time
                print (self.time, "Starting to track %s" % artifact_type)
                self.bus.publish('driving_control', artifact_type)
            else:
                for looking_for in self.objects_to_follow:
                    if self.currently_following_object['object_type'] == looking_for: # detected artefact is top priority and it is the one being followed already, continue what you were doing
                        break 
                    elif looking_for == artifact_type: # we are looking for this artifact but it's not the one currently being followed, switch
                        print (self.time, "Switching to tracking %s" % artifact_type)
                        self.currently_following_object['object_type'] = artifact_type
                        self.bus.publish('driving_control', artifact_type)

            if self.currently_following_object['object_type'] == artifact_type:
                self.currently_following_object['timestamp'] = self.time

                if self.currently_following_object['object_type'] == 'cubesat':
                    #            print("Cubesat reported at %d %d %d %d" % (data[1], data[2], data[3], data[4]))
                    # virtual bumper still applies while this block has control. When triggered, driving will go to recovery and main will take over driving

                    # when cubesat disappears, we need to reset the steering to going straight
                    # NOTE: light does not shine in corners of viewport, need to report sooner or turn first
                    if bbox_size > 25 and data[2] < 40: # box is big enough to report on and close to the edge, report
                         # box 25 pixels represents distance about 27m which is as close as we can possibly get for cubesats with high altitude 
                        # object in center (x axis) and close enough (bbox size)
                        # stop and report angle and distance from robot
                        # robot moves a little after detection so the angles do not correspond with the true pose we will receive
                        # TODO: if found during side sweep, robot will turn some between last frame and true pose messing up the angle
                        self.desired_linear_speed = 0.0
                        self.desired_angular_speed = 0.0
                        print(self.time, "rover: cubesat final frame x=%d y=%d w=%d h=%d" % (data[1], data[2], data[3], data[4]))

                        if 'homebase' in self.objects_to_follow:
                            self.objects_to_follow.remove('homebase') # do not immediately follow homebase if it was secondary to give main a chance to report cubesat
                        self.object_reached(artifact_type)
                        
                    elif center_x < 200: # if cubesat near left edge, turn left; if far enough from top, go straight too, otherwise turn in place
                        self.desired_angular_speed = SPEED_ON
                        self.desired_linear_speed = SPEED_ON if data[2] > 20 else 0.0
                    elif center_x > 440:
                        self.desired_angular_speed = -SPEED_ON
                        self.desired_linear_speed = SPEED_ON if data[2] > 20 else 0.0
                    else:
                        # bbox is ahead but too small or position not near the edge, continue straight
                        self.desired_linear_speed = SPEED_ON
                        self.desired_angular_speed = 0.0

                        
                elif not self.homebase_final_approach and self.currently_following_object['object_type'] == 'homebase':
                    if bbox_size > 200:
                        if center_x >= 300 and center_x <= 340:
                            # object reached visually, keep moving forward
                            self.desired_angular_speed = 0.0
                            self.desired_linear_speed = SPEED_ON
                            print(self.time, "homebase final frame x=%d y=%d w=%d h=%d" % (data[1], data[2], data[3], data[4]))
                            self.homebase_final_approach = True
                        elif center_x < 300: # close but wrong angle, turn in place left
                            self.desired_angular_speed = SPEED_ON
                            self.desired_linear_speed = 0.0
                        elif center_x > 340: # close but wrong angle, turn in place right
                            self.desired_angular_speed = -SPEED_ON
                            self.desired_linear_speed = 0.0
                    else:
                        if center_x < 300: # if homebase to the left, steer left
                            self.desired_angular_speed = SPEED_ON
                            self.desired_linear_speed = SPEED_ON
                        elif center_x > 340:
                            self.desired_angular_speed = -SPEED_ON
                            self.desired_linear_speed = SPEED_ON
                        else: # if within angle but object too small, keep going straight
                            self.desired_angular_speed = 0.0
                            self.desired_linear_speed = SPEED_ON

                elif self.currently_following_object['object_type'] == 'basemarker':
                    print(self.time, "rover: basemarker identified")

                    if center_x < 300: # if marker to the left
                        self.basemarker_centered = False
                    elif center_x > 340:
                        self.basemarker_centered = False
                    else:
                        self.basemarker_centered = True
                        
                        
    def on_rot(self, data):
        rot = data
        rot[2] += 18000
        (temp_yaw, self.pitch, self.roll) = [normalizeAnglePIPI(math.radians(x/100)) for x in rot]
        
        if self.yaw_offset is None:
            self.yaw_offset = -temp_yaw
        self.yaw = temp_yaw + self.yaw_offset
        #print ("yaw: %f, pitch: %f, roll: %f" % (self.yaw, self.pitch, self.roll))
        
    def on_joint_position(self, data):
        assert self.joint_name is not None
        if self.prev_position is None:
            self.prev_position = data

        diff = [b - a for a, b in zip(self.prev_position, data)]

        assert b'bl_wheel_joint' in self.joint_name, self.joint_name
        if self.desired_linear_speed >= 0:
            name = b'bl_wheel_joint'
            name2 = b'br_wheel_joint'
        else:
            name = b'fl_wheel_joint'
            name2 = b'fr_wheel_joint'
        left = WHEEL_RADIUS * diff[self.joint_name.index(name)]
        right = WHEEL_RADIUS * diff[self.joint_name.index(name2)]
        dist = (left + right)/2
        angle = (right - left)/WHEEL_SEPARATION_WIDTH
        x, y, heading = self.pose2d
        x += math.cos(heading) * dist
        y += math.sin(heading) * dist
        heading += angle
        self.bus.publish('pose2d', [round(x * 1000),
                                    round(y * 1000),
                                    round(math.degrees(heading) * 100)])
        self.prev_position = data
        self.pose2d = x, y, heading

    def on_joint_velocity(self, data):
        assert self.joint_name is not None
        speed = []
        for wheel in WHEEL_NAMES:  # cycle through fl, fr, bl, br
            speed.append(WHEEL_RADIUS * data[self.joint_name.index(bytes(wheel, 'ascii') + b'_wheel_joint')])
        if self.verbose:
            self.debug_arr.append([self.time.total_seconds(),] + speed)

    def on_scan(self, data):
        assert len(data) == 180
        # NASA sends 100 samples over 150 degrees
        # OSGAR sends 180 points, first and last 40 are zeros

# TODO: if too far from anything, revert to looking for homebase
        if not self.in_driving_recovery:

            midindex = len(data) // 2
            # 10 degrees left and right is 6-7 samples before and after the array midpoint
            straight_ahead_dist = min_dist(data[midindex-15:midindex+15])

            if 'homebase' in self.objects_to_follow and self.homebase_final_approach:
                if straight_ahead_dist < HOMEBASE_KEEP_DISTANCE:
                    cmd = b'cmd_rover 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0'
                    self.bus.publish('cmd', cmd)

                    print ("rover: homebase distance %f: " % straight_ahead_dist)
                    self.homebase_final_approach = False
                    self.object_reached('homebase')
                else:
                    # keep going straight; for now this means do nothing, keep speed from previous step
                    # if it misses the base, will keep going until it hits something, then it will quit claiming it found the base
                    # print("rover: Keeping going")
                    self.currently_following_object['timestamp'] = self.time # freshen up timer as we are still following homebase

            # we expect that lidar bounces off of homebase as if it was a big cylinder, not taking into consideration legs, etc.

            if 'basemarker' in self.objects_to_follow:
                right_dist = median_dist(data[midindex-8:midindex-6])
                left_dist = median_dist(data[midindex+6:midindex+8])
                print ("rover: Min dist front: %f, dist left=%f, right=%f" % (straight_ahead_dist, left_dist, right_dist))

                e = 80
                if self.started_turning_for_basemarker_timestamp == None:
                    self.started_turning_for_basemarker_timestamp = self.time
                elif self.time - self.started_turning_for_basemarker_timestamp < timedelta(seconds=4):
                    e = 0

    #            if left_dist < 9:
    #                print ("rover: right / left distance ratio: %f; centered: %r" % (right_dist / left_dist, self.basemarker_centered))
                if self.basemarker_centered and left_dist < 6 and abs(1.0 - right_dist / left_dist) < 0.04: # cos 20 = dist_r / dist _l is the max ratio in order to be at most 10 degrees off; also needs to be closer than 6m
                    cmd = b'cmd_rover 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0'
                    self.bus.publish('cmd', cmd)
                    self.started_turning_for_basemarker_timestamp = None
                    self.object_reached('basemarker')
                    return

                effort = [e, -e, e, -e]

                # estimated radius of homebase is 4m
                # rover width is 2.2m
                steering_angle_left = steering_angle_right = math.atan((4 + HOMEBASE_KEEP_DISTANCE) / 1.1)

                if straight_ahead_dist >  1.1 * HOMEBASE_KEEP_DISTANCE:
                        steering_angle_left -= 0.1
                        steering_angle_right += 0.1
                elif straight_ahead_dist <  0.9 * HOMEBASE_KEEP_DISTANCE:
                        steering_angle_left += 0.1
                        steering_angle_right -= 0.1
                elif right_dist > 15.0: # overshot turn, right view is past homebase
                        steering_angle_left += 0.1
                        steering_angle_right -= 0.1
                elif left_dist > 0.01 and right_dist > 0.01:
                    if left_dist < 0.9 * right_dist:  # tighten turn
                        steering_angle_left -= 0.1
                        steering_angle_right += 0.1
                    elif left_dist * 0.9 > right_dist: # loosen turn
                        steering_angle_left += 0.1
                        steering_angle_right -= 0.1

                steering = [steering_angle_left, -steering_angle_right, steering_angle_left, -steering_angle_right]
                cmd = b'cmd_rover %f %f %f %f %f %f %f %f' % tuple(steering + effort)
                self.bus.publish('cmd', cmd)
            
    def on_joint_effort(self, data):
        assert self.joint_name is not None

        # TODO cycle through fl, fr, bl, br
        effort =  data[self.joint_name.index(b'fl_wheel_joint')]

        steering = [0.0,] * 4

        # if was following an artefact but it disappeared, just go straight until another driver takes over
        if self.currently_following_object['timestamp'] is not None and self.time - self.currently_following_object['timestamp'] > self.object_timeouts[self.currently_following_object['object_type']]:
            self.desired_linear_speed = SPEED_ON
            self.desired_angular_speed = 0.0
            self.bus.publish('driving_control', None)
            print (self.time, "No longer tracking %s" % self.currently_following_object['object_type'])
            self.currently_following_object['timestamp'] = None
            self.currently_following_object['object_type'] = None
            self.basemarker_centered = False


        # do not control wheels if driven via basemarker search
        if 'basemarker' in self.objects_to_follow: 
            return
            
        # only turning
        if abs(self.desired_linear_speed) < 0.001:
            e = 40 if self.last_artefact_time is None else 20 # when turning to adjust to follow object, do it slowly to receive feedback from cameras; this shouldn't actually happen as we steer and go together when following an object
            if abs(self.desired_angular_speed) < 0.001:
                effort = [0,] * 4
            elif self.desired_angular_speed > 0:
                # turn left
                effort = [-e, e, -e, e]
                steering = [-CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,-CRAB_ROLL_ANGLE]
            else:
                # turn right
                effort = [e, -e, e, -e]
                steering = [-CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,-CRAB_ROLL_ANGLE]

        elif self.desired_linear_speed > 0: # going forward

            # if pitch too steep, turn diagonally a try to climb this way being able to handle larger pitch
            if self.pitch > 0.3:
                # TODO: try to maintain wheel orientation up (wheels turning straight up even if body turns)
                if self.roll > 0:
                    #steering = [3*-self.roll,3*-self.roll,0.0, 0.0]

                    steering = [-CRAB_ROLL_ANGLE,-CRAB_ROLL_ANGLE,-CRAB_ROLL_ANGLE,-CRAB_ROLL_ANGLE]
                else:
                    #steering = [3*-self.roll,3*-self.roll,0.0, 0.0]
                    steering = [CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE,CRAB_ROLL_ANGLE]
                e = e2 = 120
            else:

                steering_angle = 0.0
                if self.desired_angular_speed > 0.001: # want to go both forward and steer, this currently only happens when following an object
                    steering_angle = STEER_TOWARDS_OBJECT_ANGLE
                elif self.desired_angular_speed < -0.001:
                    steering_angle = -STEER_TOWARDS_OBJECT_ANGLE

                # steer against slope proportionately to the steepness of the slope
                steering_angle -= self.roll
                # dont steer more than 60 degrees
                steering_angle = max(-math.pi/2.0, steering_angle)
                steering_angle = min(math.pi/2.0, steering_angle)
                steering = [steering_angle, steering_angle, steering_angle / 4, steering_angle / 4]
                
                e = 80 if self.pitch > FOUR_WHEEL_DRIVE_PITCH_THRESHOLD else 120
                e2 = e if self.pitch > FOUR_WHEEL_DRIVE_PITCH_THRESHOLD else e # 4wd all the time
            effort = [e, e, e2, e2]
        else: # going backward
            e = 80
            effort = [-e, -e, -e, -e]
        cmd = b'cmd_rover %f %f %f %f %f %f %f %f' % tuple(steering + effort)
        self.bus.publish('cmd', cmd)

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

        return channel


    def draw(self):
        # for debugging
        import matplotlib.pyplot as plt
        arr = self.debug_arr
        t = [a[0] for a in arr]
        values = [a[1:] for a in arr]

        line = plt.plot(t, values, '-', linewidth=2)

        plt.xlabel('time (s)')
        plt.legend(WHEEL_NAMES)
        plt.show()

# vim: expandtab sw=4 ts=4
