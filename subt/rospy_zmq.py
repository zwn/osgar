"""
  Wait for all necessary ROS sensors
"""
import time
import struct
from io import BytesIO

import zmq

import rospy
from sensor_msgs.msg import Imu, LaserScan, CompressedImage, BatteryState
from nav_msgs.msg import Odometry


ROBOT_NAME = 'X0F200L'


def wait_for_master():
    # it looks like master is not quite ready for several minutes and the only indication is the list of published
    # topics
    rospy.loginfo('wait_for_master')
    while True:
        for topic, topic_type in rospy.get_published_topics():
            if 'battery_state' in topic:
                rospy.loginfo('found ' + topic)
                return topic
        time.sleep(0.1)


def wait_for(topic, topic_type):
    rospy.loginfo('Wait for ' + topic)
    rospy.wait_for_message(topic, topic_type)
    rospy.loginfo('Done with ' + topic)


def wait_for_sensors():
    rospy.init_node('mdwait', anonymous=True)
    wait_for_master()
    rospy.loginfo('-------------- mdwait BEGIN --------------')
    wait_for('/'+ROBOT_NAME+'/imu/data', Imu)
    wait_for('/'+ROBOT_NAME+'/front_scan', LaserScan)
    wait_for('/'+ROBOT_NAME+'/front/image_raw/compressed', CompressedImage)
    wait_for('/'+ROBOT_NAME+'/odom', Odometry)
    wait_for('/'+ROBOT_NAME+'/battery_state', BatteryState)  # note, that this is maybe the critical component!
    rospy.loginfo('--------------- mdwait END ---------------')


g_socket = None

def callback(data):
    global g_socket
    assert g_socket is not None

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#    print(rospy.get_caller_id(), data)

    # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
    s1 = BytesIO()
    data.serialize(s1)
    to_send = s1.getvalue()
    header = struct.pack('<I', len(to_send))
    g_socket.send(header + to_send)


def odom2zmq():
    global g_socket
    wait_for_master()

    context = zmq.Context()
    g_socket = context.socket(zmq.PUSH)
    g_socket.setsockopt(zmq.LINGER, 100)  # milliseconds
    g_socket.bind('tcp://*:5555')

    context2 = zmq.Context()
    g_socket2 = context2.socket(zmq.PULL)
    g_socket2.RCVTIMEO = 5000 # in milliseconds
    g_socket2.bind('tcp://*:5556')
  
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/'+ROBOT_NAME+'/odom', Odometry, callback)

    while True:
        try:
            message = g_socket2.recv()
            print(message)
        except zmq.error.Again:
            pass


if __name__ == '__main__':
    wait_for_master()
    odom2zmq()

# vim: expandtab sw=4 ts=4
