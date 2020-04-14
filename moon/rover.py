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

from osgar.node import Node


WHEEL_RADIUS = 0.275  # meters


class Rover(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("cmd")
        self.desired_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0
        self.joint_name = None  # updated via Node.update()

    def on_desired_speed(self, data):
        self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)

    def on_joint_velocity(self, data):
        assert self.joint_name is not None
        # TODO cycle through fl, fr, bl, br
        speed = WHEEL_RADIUS * data[self.joint_name.index(b'fl_wheel_joint')]
#        print("%.3f" % speed)  # TODO change to string

    def on_joint_effort(self, data):
        assert self.joint_name is not None
        # TODO cycle through fl, fr, bl, br
        effort =  data[self.joint_name.index(b'fl_wheel_joint')]
        print("%.3f" % effort)

        # workaround for not existing /clock on Moon rover
        steering = [0,] * 4
        if abs(self.desired_speed) < 0.001:
            effort = [0,] * 4
        else:
            effort = [100,] * 4
        cmd = b'cmd_rover %f %f %f %f %f %f %f %f' % tuple(steering + effort)
        self.bus.publish('cmd', cmd)

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

        return channel

# vim: expandtab sw=4 ts=4
