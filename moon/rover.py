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



# vim: expandtab sw=4 ts=4
