#!/usr/bin/python
"""
  Wait for all necessary ROS sensors
"""
import time
import struct
from io import BytesIO

import zmq

import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist

ROBOT_NAME = 'scout_1'
FILTER_ODOM_NTH = 10 #n - every nth message shall be sent to osgar
FILTER_CAMERA_NTH = 4 #n - every nth message shall be sent to osgar
FILTER_DEPTH_NTH = 10000000 #n - every nth message shall be sent to osgar
g_odom_counter = 0
g_depth_counter = 0
g_camera_counter = 0


g_socket = None

def callback(data):
    global g_socket
    assert g_socket is not None

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # print(rospy.get_caller_id(), data)

    # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
    s1 = BytesIO()
    data.serialize(s1)
    to_send = s1.getvalue()
    header = struct.pack('<I', len(to_send))
    g_socket.send(header + to_send)


def callback_imu(data):
    global g_socket
    assert g_socket is not None

    s1 = BytesIO()
    data.serialize(s1)
    to_send = s1.getvalue()
    header = struct.pack('<I', len(to_send))
    g_socket.send(header + to_send)


def callback_odom(data):
    global g_socket, g_odom_counter
    assert g_socket is not None

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # print(rospy.get_caller_id(), data)

    # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
    if g_odom_counter >= FILTER_ODOM_NTH:
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        g_socket.send(header + to_send)
        g_odom_counter = 0
    else:
        g_odom_counter += 1


def callback_depth(data):
    global g_socket, g_depth_counter
    assert g_socket is not None

    # rospy.loginfo(rospy.get_caller_id() + "I heard depth data")
    # print(rospy.get_caller_id(), data)

    # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
    if g_depth_counter >= FILTER_DEPTH_NTH:
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        g_socket.send("depth" + header + to_send)
        g_depth_counter = 0
    else:
        g_depth_counter += 1


def callback_camera(data):
    global g_socket, g_camera_counter
    assert g_socket is not None

    # rospy.loginfo(rospy.get_caller_id() + "I heard depth data")
    # print(rospy.get_caller_id(), data)

    # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
    if g_camera_counter >= FILTER_CAMERA_NTH:
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        g_socket.send(header + to_send)
        g_camera_counter = 0
    else:
        g_camera_counter += 1


def callback_clock(data):
    global g_socket
    assert g_socket is not None

    s1 = BytesIO()
    data.serialize(s1)
    to_send = s1.getvalue()
    header = struct.pack('<I', len(to_send))
    g_socket.send(header + to_send)


def odom2zmq():
    global g_socket
    #wait_for_master()

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
    rospy.Subscriber('/scout_1/laser/scan', LaserScan, callback)
    rospy.Subscriber('/scout_1/imu', Imu, callback_imu)
#    rospy.Subscriber('/depth_image', Image, callback_depth)
#    rospy.Subscriber('/image', CompressedImage, callback_camera)
#    rospy.Subscriber('/clock', Clock, callback_clock)
    
    
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    r = rospy.Rate(10)
    while True:
        try:
            message = ""
            try:
                while 1:
                    message = g_socket2.recv(zmq.NOBLOCK)
            except:
                pass
            #print("OSGAR:" + message)
            if message.split(" ")[0] == "cmd_vel":
                vel_msg.linear.x = float(message.split(" ")[1])
                vel_msg.angular.z = float(message.split(" ")[2])
                velocity_publisher.publish(vel_msg)
        except zmq.error.Again:
            pass
        r.sleep()


if __name__ == '__main__':
    odom2zmq()

# vim: expandtab sw=4 ts=4
