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


# SCOUT
# Volatile Sensor
# /scout_n/volatile_sensor  srcp2_msgs/vol_sensor_msg


# EXCAVATOR
# Excavator Arm and Bucket
# excavator_n/bucket_info  srcp2_msgs/excavator_msg
# excavator_n/mount_joint_controller/command  - 360 degrees
# excavator_n/basearm_joint_controller/command  - largest part
# excavator_n/distalarm_joint_controller/command
# excavator_n/bucket_joint_controller/command  - last bit "backet"


# HAULER
# /hauler_n/bin_info  srcp2_msgs/hauler_msg
# /hauler_n/bin_joint_controller/command

import math
from osgar.lib.mathex import normalizeAnglePIPI

from osgar.node import Node


WHEEL_RADIUS = 0.275  # meters
WHEEL_SEPARATION_WIDTH = 1.87325  # meters
WHEEL_SEPARATION_HEIGHT = 1.5748  # meters

WHEEL_NAMES = ['fl', 'fr', 'bl', 'br']

FOUR_WHEEL_DRIVE_PITCH_THRESHOLD = 0.1
CRAB_DRIVE_ROLL_THRESHOLD = 0.4
CRAB_ROLL_ANGLE = 0.78

class Rover(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('cmd', 'pose2d')
        self.desired_speed = 0.0  # m/s
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

    def on_desired_speed(self, data):
        self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)
        
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
        if self.desired_speed >= 0:
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

    def on_joint_effort(self, data):
        assert self.joint_name is not None
        # TODO cycle through fl, fr, bl, br
        effort =  data[self.joint_name.index(b'fl_wheel_joint')]

        steering = [0.0,] * 4

        # workaround for not existing /clock on Moon rover
        
        if abs(self.desired_speed) < 0.001:
            e = 80
            if abs(self.desired_angular_speed) < 0.001:
                effort = [0,] * 4
            elif self.desired_angular_speed > 0:
                # turn left
                effort = [-e, e, -e, e]
            else:
                # turn right
                effort = [e, -e, e, -e]
        elif self.desired_speed > 0:
            is_crab = False
            if self.roll > CRAB_DRIVE_ROLL_THRESHOLD:
                steering = [-CRAB_ROLL_ANGLE, -CRAB_ROLL_ANGLE, 0.0, 0.0 ]
                is_crab = True
            elif self.roll < -CRAB_DRIVE_ROLL_THRESHOLD:
                steering = [CRAB_ROLL_ANGLE, CRAB_ROLL_ANGLE, 0.0, 0.0]
                is_crab = True
            else:
                steering = [0.0,] * 4
            e = 80
            e2 = e if self.pitch > FOUR_WHEEL_DRIVE_PITCH_THRESHOLD else 0.0
            effort = [e, e, e2, e2]
        else:
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
