#!/usr/bin/python
"""
  Wait for all necessary ROS sensors
"""
import time
import struct
import math
from io import BytesIO
from threading import RLock

import zmq

import rospy
from std_msgs.msg import *  # Float64, JointState
from sensor_msgs.msg import *
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, Point

# SRCP2 specific
from srcp2_msgs.msg import Qual1ScoringMsg, VolSensorMsg, Qual3ScoringMsg
from srcp2_msgs.srv import (ToggleLightSrv, BrakeRoverSrv, LocalizationSrv, Qual1ScoreSrv,
                            AprioriLocationSrv, HomeLocationSrv, HomeAlignedSrv)


ROBOT_NAME = 'scout_1'

WHEEL_SEPARATION_WIDTH = 1.87325  # meters
WHEEL_SEPARATION_HEIGHT = 1.5748  # meters


FILTER_ODOM_NTH = 1  #n - every nth message shall be sent to osgar
FILTER_CAMERA_NTH = 4 #n - every nth message shall be sent to osgar
FILTER_DEPTH_NTH = 1  #n - every nth message shall be sent to osgar
g_odom_counter = 0
g_depth_counter = 0
g_camera_counter = 0


g_socket = None
g_lock = RLock()


def socket_send(data):
    global g_socket, g_lock
    assert g_socket is not None
    with g_lock:
        g_socket.send(data)


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # print(rospy.get_caller_id(), data)

    # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
    s1 = BytesIO()
    data.serialize(s1)
    to_send = s1.getvalue()
    header = struct.pack('<I', len(to_send))
    socket_send(header + to_send)


def callback_imu(data):
    s1 = BytesIO()
    data.serialize(s1)
    to_send = s1.getvalue()
    header = struct.pack('<I', len(to_send))
    socket_send(header + to_send)


def callback_odom(data):
    global g_odom_counter
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # print(rospy.get_caller_id(), data)

    # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
    if g_odom_counter >= FILTER_ODOM_NTH:
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        socket_send(header + to_send)
        g_odom_counter = 0
    else:
        g_odom_counter += 1


def callback_depth(data):
    global g_depth_counter
    # rospy.loginfo(rospy.get_caller_id() + "I heard depth data")
    # print(rospy.get_caller_id(), data)

    # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
    if g_depth_counter >= FILTER_DEPTH_NTH:
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        socket_send("depth" + header + to_send)
        g_depth_counter = 0
    else:
        g_depth_counter += 1


def callback_camera(data):
    global g_camera_counter
    # rospy.loginfo(rospy.get_caller_id() + "I heard depth data")
    # print(rospy.get_caller_id(), data)

    # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
    if g_camera_counter >= FILTER_CAMERA_NTH:
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        socket_send(header + to_send)
        g_camera_counter = 0
    else:
        g_camera_counter += 1


def callback_clock(data):
    s1 = BytesIO()
    data.serialize(s1)
    to_send = s1.getvalue()
    header = struct.pack('<I', len(to_send))
    socket_send(header + to_send)


def callback_topic(data, topic_name):
    s1 = BytesIO()
    data.serialize(s1)
    to_send = s1.getvalue()
    header = struct.pack('<I', len(to_send))
    socket_send(topic_name + '\0' + header + to_send)


def odom2zmq():
    global g_socket, g_lock

    with g_lock:
        context = zmq.Context()
        g_socket = context.socket(zmq.PUSH)
        g_socket.setsockopt(zmq.LINGER, 100)  # milliseconds
        g_socket.bind('tcp://*:5555')

    context2 = zmq.Context()
    g_socket2 = context2.socket(zmq.PULL)
    g_socket2.RCVTIMEO = 5000 # in milliseconds
    g_socket2.bind('tcp://*:5556')
  
    rospy.init_node('listener', anonymous=True)
#    rospy.Subscriber('/odom', Odometry, callback_odom)
    rospy.Subscriber('/scout_1/joint_states', JointState, callback_topic, '/scout_1/joint_states')
    rospy.Subscriber('/scout_1/laser/scan', LaserScan, callback)
    rospy.Subscriber('/scout_1/imu', Imu, callback_imu)
#    rospy.Subscriber('/scout_1/camera/left/image_raw', Image, callback_depth)
#    rospy.Subscriber('/image', CompressedImage, callback_camera)
#    rospy.Subscriber('/clock', Clock, callback_clock)

    QSIZE = 10

    lights_on = rospy.ServiceProxy('/scout_1/toggle_light', ToggleLightSrv)
    lights_on('high')

    light_up_pub = rospy.Publisher('/scout_1/sensor_controller/command', Float64, queue_size=QSIZE, latch=True)
    light_up_msg = Float64()

    brakes = rospy.ServiceProxy('/scout_1/brake_rover', BrakeRoverSrv)
    
    # TODO load it from configuration
    # task 1
    rospy.Subscriber('/qual_1_score', Qual1ScoringMsg, callback_topic, '/qual_1_score')
    rospy.Subscriber('/scout_1/volatile_sensor', VolSensorMsg, callback_topic, '/scout_1/volatile_sensor')

    # task 3
    rospy.Subscriber('/qual_3_score', Qual3ScoringMsg, callback_topic, '/qual_3_score')

    rospy.Subscriber('/scout_1/camera/left/image_raw/compressed', CompressedImage, callback_topic, 
                     '/scout_1/camera/left/image_raw/compressed')
    rospy.Subscriber('/scout_1/camera/right/image_raw/compressed', CompressedImage, callback_topic, 
                     '/scout_1/camera/right/image_raw/compressed')

#    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    speed_msg = Float64()
    speed_msg.data = 0

    steering_msg = Float64()
    steering_msg.data = 0
    effort_msg = Float64()
    effort_msg.data = 0

    # /name/fl_wheel_controller/command
    vel_fl_publisher = rospy.Publisher('/scout_1/fl_wheel_controller/command', Float64, queue_size=QSIZE)
    vel_fr_publisher = rospy.Publisher('/scout_1/fr_wheel_controller/command', Float64, queue_size=QSIZE)
    vel_bl_publisher = rospy.Publisher('/scout_1/bl_wheel_controller/command', Float64, queue_size=QSIZE)
    vel_br_publisher = rospy.Publisher('/scout_1/br_wheel_controller/command', Float64, queue_size=QSIZE)

    steering_fl_publisher = rospy.Publisher('/scout_1/fl_steering_arm_controller/command', Float64, queue_size=QSIZE)
    steering_fr_publisher = rospy.Publisher('/scout_1/fr_steering_arm_controller/command', Float64, queue_size=QSIZE)
    steering_bl_publisher = rospy.Publisher('/scout_1/bl_steering_arm_controller/command', Float64, queue_size=QSIZE)
    steering_br_publisher = rospy.Publisher('/scout_1/br_steering_arm_controller/command', Float64, queue_size=QSIZE)

    r = rospy.Rate(100)
    while True:
        try:
            message = ""
            try:
                while 1:
                    message = g_socket2.recv(zmq.NOBLOCK)
            except:
                pass
            #print("OSGAR:" + message)
            message_type = message.split(" ")[0]
            if message_type == "cmd_rover":
                arr = [float(x) for x in message.split()[1:]]
                for pub, angle in zip(
                        [steering_fl_publisher, steering_fr_publisher, steering_bl_publisher, steering_br_publisher],
                        arr[:4]):
                    steering_msg.data = angle
                    pub.publish(steering_msg)

                for pub, effort in zip(
                        [vel_fl_publisher, vel_fr_publisher, vel_bl_publisher, vel_br_publisher],
                        arr[4:]):
                    effort_msg.data = effort
                    pub.publish(effort_msg)

            elif message_type == "set_cam_angle":
                angle = float(message.split(" ")[1])
                light_up_msg.data = angle
                light_up_pub.publish(light_up_msg)
                
            elif message_type == "set_brakes":
                is_on = message.split(" ")[1].startswith("on")
                print ("Setting brakes to: %r" % is_on)
                brakes(is_on)

            elif message_type == "cmd_vel":
                desired_speed = float(message.split(" ")[1])
                desired_angular_speed = float(message.split(" ")[2])

                if abs(desired_speed) < 0.001 and abs(desired_angular_speed) < 0.001:
                    speed_msg.data = 0
                else:
                    speed_msg.data = 100  # force to move forward, always TODO PID with scale to Nm
                vel_fl_publisher.publish(speed_msg)
                vel_fr_publisher.publish(speed_msg)
                vel_bl_publisher.publish(speed_msg)
                vel_br_publisher.publish(speed_msg)

                if abs(desired_speed) > 0.001:
                    if abs(desired_angular_speed) > 0.001:
                        # i.e. turn and go
                        radius = desired_speed/desired_angular_speed
                        angle_left = math.atan2(WHEEL_SEPARATION_HEIGHT/2.0, radius - WHEEL_SEPARATION_WIDTH/2.0)
                        angle_right = math.atan2(WHEEL_SEPARATION_HEIGHT/2.0, radius + WHEEL_SEPARATION_WIDTH/2.0)
                    else:
                        angle_left = 0.0
                        angle_right = 0.0

                    steering_msg.data = angle_left
                    steering_fl_publisher.publish(steering_msg)
                    steering_msg.data = angle_right
                    steering_fr_publisher.publish(steering_msg)
                    steering_msg.data = -angle_left
                    steering_bl_publisher.publish(steering_msg)
                    steering_msg.data = -angle_right
                    steering_br_publisher.publish(steering_msg)
                else:
                    pass  # keep steering angles as they are ...
            elif message_type == "request_origin":
                print "Requesting true pose"
                try:
                    rospy.wait_for_service("/scout_1/get_true_pose", timeout=2.0)
                    request_origin = rospy.ServiceProxy('/scout_1/get_true_pose', LocalizationSrv)
                    p = request_origin(True)
                    print("rospy_rover: true pose [%f, %f, %f]  [%f, %f, %f, %f]" % (p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w))
                    s = "origin scout_1 %f %f %f  %f %f %f %f" % (p.pose.position.x, p.pose.position.y, p.pose.position.z, 
                                                                  p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w)
                    socket_send(s)
                except rospy.service.ServiceException as e:
                    print(e)
            elif message_type == "artf":
                s = message.split()[1:]  # ignore "artf" prefix
                vol_type = s[0]
                if vol_type == 'cubesat':
                    # Task 3
                    x, y, z = [float(a) for a in s[1:]]
                    pose = geometry_msgs.msg.Point(x, y, z)
                    print ("rospy_rover: Reporting %s at position %f %f %f" % (vol_type, x, y, z))
                    try:
                        rospy.wait_for_service("/apriori_location_service", timeout=2.0)
                        report_artf = rospy.ServiceProxy('/apriori_location_service', AprioriLocationSrv)
                        print(report_artf(pose))
                    except rospy.service.ServiceException as e:
                        print(e)
                    except rospy.ROSException as exc:
                        print("/apriori_location_service not available: " + str(exc))
                elif vol_type == 'homebase':
                    # Task 3
                    print("rospy_rover: reporting homebase arrival")
                    rospy.wait_for_service("/arrived_home_service", timeout=2.0)
                    report_artf = rospy.ServiceProxy('/arrived_home_service', HomeLocationSrv)
                    try:
                        print(report_artf(True))
                    except rospy.service.ServiceException as e:
                        print(e)
                    except rospy.ROSException as exc:
                        print("/arrived_home_service not available: " + str(exc))
                elif vol_type == 'homebase_alignment':
                    # Task 3
                    rospy.wait_for_service("/aligned_service", timeout=2.0)
                    report_artf = rospy.ServiceProxy('/aligned_service', HomeLocationSrv)
                    try:
                        print(report_artf(True))
                    except rospy.service.ServiceException as e:
                        print(e)
                    except rospy.ROSException as exc:
                        print("/aligned_service not available: " + str(exc))
                elif vol_type in ['ice', 'ethene', 'methane', 'methanol', 'carbon_dio', 'ammonia', 'hydrogen_sul', 'sulfur_dio']:
                    # Task 1
                    x, y, z = [float(a) for a in s[1:]]
                    pose = geometry_msgs.msg.Point(x, y, z)
                    print ("Reporting %s at position %f %f %f" % (vol_type, x, y, z))
                    try:
                        rospy.wait_for_service("/vol_detected_service", timeout=2.0)
                        report_artf = rospy.ServiceProxy('/vol_detected_service', Qual1ScoreSrv)
                        resp = report_artf(pose=pose, vol_type=vol_type)
                        print ("Volatile report result: %r" % resp.result)
                    except rospy.ServiceException as exc:
                        print("/vol_detected_service exception: " + str(exc))
                    except rospy.ROSException as exc:
                        if False:
                            print("/vol_detected_service not available: " + str(exc))
            else:
                if len(message_type) > 0: 
                    print ("Unhandled message type: %s" % message_type)

        except zmq.error.Again:
            pass
        r.sleep()


if __name__ == '__main__':
    odom2zmq()

# vim: expandtab sw=4 ts=4
