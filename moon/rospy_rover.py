#!/usr/bin/python
"""
  Wait for all necessary ROS sensors
  this is Python 2.7 code
"""
import time
import struct
import math
from io import BytesIO

import threading
from threading import RLock
from threading import Thread

import signal

import zmq
import sys, getopt

import rospy
from std_msgs.msg import *  # Float64, JointState
from sensor_msgs.msg import *
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, Point

# SRCP2 specific
from srcp2_msgs.msg import Qual1ScoringMsg, Qual2ScoringMsg, Qual3ScoringMsg
from srcp2_msgs.srv import (ToggleLightSrv, BrakeRoverSrv, LocalizationSrv, Qual1ScoreSrv,
                            AprioriLocationSrv, HomeLocationSrv, HomeAlignedSrv)


interrupted = False

    
def signal_handler(signum, frame):
    global interrupted
    interrupted = True



class RospyRoverPushPull(Thread):
    def __init__(self, robot_name, push_port, pull_port):
        Thread.__init__(self)
        self.PUSH_PORT = push_port
        self.PULL_PORT = pull_port
        self.robot_name = robot_name
        self.pull_socket = None

        self.WHEEL_SEPARATION_WIDTH = 1.87325  # meters
        self.WHEEL_SEPARATION_HEIGHT = 1.5748  # meters

        self.FILTER_ODOM_NTH = 1  #n - every nth message shall be sent to osgar
        self.FILTER_CAMERA_NTH = 4 #n - every nth message shall be sent to osgar
        self.FILTER_DEPTH_NTH = 1  #n - every nth message shall be sent to osgar

        self.g_odom_counter = 0
        self.g_depth_counter = 0
        self.g_camera_counter = 0


        self.g_socket = None
        self.g_lock = RLock()


        
    def setup_sockets(self, context=None):

        with self.g_lock:
            context = context or zmq.Context()
            self.g_socket = context.socket(zmq.PUSH)
            self.g_socket.setsockopt(zmq.LINGER, 0)  # milliseconds
            self.g_socket.bind('tcp://*:' + self.PUSH_PORT)

        context2 = context or zmq.Context()
        self.pull_socket = context2.socket(zmq.PULL)
        self.pull_socket.setsockopt(zmq.LINGER, 0)  # milliseconds
        self.pull_socket.RCVTIMEO = 2000 
        self.pull_socket.bind('tcp://*:' + self.PULL_PORT)

       
        
        
    def register_handlers(self):
    #    rospy.init_node('listener', anonymous=True)
    #    rospy.Subscriber('/odom', Odometry, callback_odom)

        rospy.Subscriber('/' + self.robot_name + '/joint_states', JointState, self.callback_topic, '/' + self.robot_name + '/joint_states')
        rospy.Subscriber('/' + self.robot_name + '/laser/scan', LaserScan, self.callback)
        rospy.Subscriber('/' + self.robot_name + '/imu', Imu, self.callback_imu)
    #    rospy.Subscriber('/' + self.robot_name + '/camera/left/image_raw', Image, callback_depth)
    #    rospy.Subscriber('/image', CompressedImage, callback_camera)
    #    rospy.Subscriber('/clock', Clock, callback_clock)

        QSIZE = 10

        rospy.Subscriber('/qual_1_score', Qual1ScoringMsg, self.callback_topic, '/qual_1_score')
        rospy.Subscriber('/qual_2_score', Qual1ScoringMsg, self.callback_topic, '/qual_2_score')
        rospy.Subscriber('/qual_3_score', Qual3ScoringMsg, self.callback_topic, '/qual_3_score')

        rospy.Subscriber('/' + self.robot_name + '/camera/left/image_raw/compressed', CompressedImage, self.callback_topic, 
                         '/' + self.robot_name + '/camera/left/image_raw/compressed')
        rospy.Subscriber('/' + self.robot_name + '/camera/right/image_raw/compressed', CompressedImage, self.callback_topic, 
                         '/' + self.robot_name + '/camera/right/image_raw/compressed')

        self.speed_msg = Float64()
        self.speed_msg.data = 0

        self.steering_msg = Float64()
        self.steering_msg.data = 0
        self.effort_msg = Float64()
        self.effort_msg.data = 0

        # /name/fl_wheel_controller/command
        self.vel_fl_publisher = rospy.Publisher('/' + self.robot_name + '/fl_wheel_controller/command', Float64, queue_size=QSIZE)
        self.vel_fr_publisher = rospy.Publisher('/' + self.robot_name + '/fr_wheel_controller/command', Float64, queue_size=QSIZE)
        self.vel_bl_publisher = rospy.Publisher('/' + self.robot_name + '/bl_wheel_controller/command', Float64, queue_size=QSIZE)
        self.vel_br_publisher = rospy.Publisher('/' + self.robot_name + '/br_wheel_controller/command', Float64, queue_size=QSIZE)

        self.steering_fl_publisher = rospy.Publisher('/' + self.robot_name + '/fl_steering_arm_controller/command', Float64, queue_size=QSIZE)
        self.steering_fr_publisher = rospy.Publisher('/' + self.robot_name + '/fr_steering_arm_controller/command', Float64, queue_size=QSIZE)
        self.steering_bl_publisher = rospy.Publisher('/' + self.robot_name + '/bl_steering_arm_controller/command', Float64, queue_size=QSIZE)
        self.steering_br_publisher = rospy.Publisher('/' + self.robot_name + '/br_steering_arm_controller/command', Float64, queue_size=QSIZE)



    def process_message(self, message):
#        print("OSGAR:" + message)
        message_type = message.split(" ")[0]
        if message_type == "cmd_rover":
            arr = [float(x) for x in message.split()[1:]]
            for pub, angle in zip(
                    [self.steering_fl_publisher, self.steering_fr_publisher, self.steering_bl_publisher, self.steering_br_publisher],
                    arr[:4]):
                self.steering_msg.data = angle
                pub.publish(self.steering_msg)

            for pub, effort in zip(
                    [self.vel_fl_publisher, self.vel_fr_publisher, self.vel_bl_publisher, self.vel_br_publisher],
                    arr[4:]):
                self.effort_msg.data = effort
                pub.publish(self.effort_msg)

        elif message_type == "cmd_vel":
            desired_speed = float(message.split(" ")[1])
            desired_angular_speed = float(message.split(" ")[2])

            if abs(desired_speed) < 0.001 and abs(desired_angular_speed) < 0.001:
                self.speed_msg.data = 0
            else:
                self.speed_msg.data = 100  # force to move forward, always TODO PID with scale to Nm
            self.vel_fl_publisher.publish(self.speed_msg)
            self.vel_fr_publisher.publish(self.speed_msg)
            self.vel_bl_publisher.publish(self.speed_msg)
            self.vel_br_publisher.publish(self.speed_msg)

            if abs(desired_speed) > 0.001:
                if abs(desired_angular_speed) > 0.001:
                    # i.e. turn and go
                    radius = desired_speed/desired_angular_speed
                    angle_left = math.atan2(WHEEL_SEPARATION_HEIGHT/2.0, radius - WHEEL_SEPARATION_WIDTH/2.0)
                    angle_right = math.atan2(WHEEL_SEPARATION_HEIGHT/2.0, radius + WHEEL_SEPARATION_WIDTH/2.0)
                else:
                    angle_left = 0.0
                    angle_right = 0.0

                self.steering_msg.data = angle_left
                self.steering_fl_publisher.publish(steering_msg)
                self.steering_msg.data = angle_right
                self.steering_fr_publisher.publish(steering_msg)
                self.steering_msg.data = -angle_left
                self.steering_bl_publisher.publish(steering_msg)
                self.steering_msg.data = -angle_right
                self.steering_br_publisher.publish(steering_msg)
            else:
                pass  # keep steering angles as they are ...
        else:
            # may be picked up by a subclass
            pass



    def socket_send(self, data):
        assert self.g_socket is not None
        with self.g_lock:
            self.g_socket.send(data)


    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        # print(rospy.get_caller_id(), data)

        # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        self.socket_send(header + to_send)


    def callback_imu(self, data):
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        self.socket_send(header + to_send)


    def callback_odom(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        # print(rospy.get_caller_id(), data)

        # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
        if self.g_odom_counter >= self.FILTER_ODOM_NTH:
            s1 = BytesIO()
            data.serialize(s1)
            to_send = s1.getvalue()
            header = struct.pack('<I', len(to_send))
            self.socket_send(header + to_send)
            g_odom_counter = 0
        else:
            g_odom_counter += 1


    def callback_depth(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard depth data")
        # print(rospy.get_caller_id(), data)

        # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
        if self.g_depth_counter >= self.FILTER_DEPTH_NTH:
            s1 = BytesIO()
            data.serialize(s1)
            to_send = s1.getvalue()
            header = struct.pack('<I', len(to_send))
            self.socket_send("depth" + header + to_send)
            self.g_depth_counter = 0
        else:
            g_depth_counter += 1


    def callback_camera(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard depth data")
        # print(rospy.get_caller_id(), data)

        # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
        if self.g_camera_counter >= self.FILTER_CAMERA_NTH:
            s1 = BytesIO()
            data.serialize(s1)
            to_send = s1.getvalue()
            header = struct.pack('<I', len(to_send))
            self.socket_send(header + to_send)
            self.g_camera_counter = 0
        else:
            self.g_camera_counter += 1


    def callback_clock(self, data):
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        self.socket_send(header + to_send)


    def callback_topic(self, data, topic_name):
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        self.socket_send(topic_name + '\0' + header + to_send)


        
    def run(self):
        
        while True:
            try:
                message = self.pull_socket.recv()
                result = self.process_message(message)
            except zmq.Again as e:
                pass

            if interrupted:
                self.pull_socket.close()
                self.g_socket.close()
                break


class RospyRoverReqRep(Thread):
    def __init__(self, robot_name, reqrep_port):
        Thread.__init__(self)
        self.REQREP_PORT = reqrep_port
        self.robot_name = robot_name
        self.reqrep_socket = None
        
    def setup_sockets(self, context=None):
        context2 = context or zmq.Context().instance()
        self.reqrep_socket = context2.socket(zmq.REP)
        self.reqrep_socket.setsockopt(zmq.LINGER, 0)  # milliseconds
        self.reqrep_socket.RCVTIMEO = 2000 
        self.reqrep_socket.bind('tcp://127.0.0.1:' + self.REQREP_PORT)

        
    def process_message(self, message):
#        print("OSGAR:" + message)
        message_type = message.split(" ")[0]
        if message_type == "set_cam_angle":
            angle = float(message.split(" ")[1])
            print ("rospy_rover: Setting cam angle to: %f" % angle)
            self.light_up_msg.data = angle
            self.light_up_pub.publish(self.light_up_msg)
            return 'OK'

        elif message_type == "set_brakes":
            is_on = message.split(" ")[1].startswith("on")
            print ("rospy_rover: Setting brakes to: %r" % is_on)
            self.brakes(is_on)
            return 'OK'

        elif message_type == "request_origin":
            print "rospy_rover: Requesting true pose"
            try:
                rospy.wait_for_service('/' + self.robot_name + '/get_true_pose', timeout=2.0)
                request_origin = rospy.ServiceProxy('/' + self.robot_name + '/get_true_pose', LocalizationSrv)
                p = request_origin(True)
                print("rospy_rover: true pose [%f, %f, %f]  [%f, %f, %f, %f]" % (p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w))
                s = "origin %f %f %f  %f %f %f %f" % (p.pose.position.x, p.pose.position.y, p.pose.position.z, 
                                                              p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w)
                return s
            except rospy.service.ServiceException as e:
                print(e)
                return str(e)
        elif message_type == "artf":
            s = message.split()[1:]  # ignore "artf" prefix
            vol_type = s[0]
            if vol_type == 'cubesat':
                # Task 3
                x, y, z = [float(a) for a in s[1:]]
                pose = geometry_msgs.msg.Point(x, y, z)
                print ("rospy_rover: Reporting %s at position %f %f %f" % (vol_type, x, y, z))
                try:
                    rospy.wait_for_service('/apriori_location_service', timeout=2.0)
                    report_artf = rospy.ServiceProxy('/apriori_location_service', AprioriLocationSrv)
                    result = report_artf(pose)
                    return 'ok'
                except rospy.service.ServiceException as e:
                    print("rospy_rover: Apriori position response: %s" % str(e))
                    return str(e)
                except rospy.ROSException as exc:
                    print("/apriori_location_service not available: " + str(exc))
                    return str(exc)
            elif vol_type == 'homebase':
                # Task 3
                print("rospy_rover: reporting homebase arrival")
                rospy.wait_for_service("/arrived_home_service", timeout=2.0)
                report_artf = rospy.ServiceProxy('/arrived_home_service', HomeLocationSrv)
                try:
                    result = report_artf(True)
                    return 'ok'
                    print("rospy_rover: Homebase arrival service response: %r" % result)
                except rospy.service.ServiceException as e:
                    print("rospy_rover: Homebase arrival service response: Incorrect")
                    return str(e)
                except rospy.ROSException as exc:
                    print("rospy_rover: /arrived_home_service not available: " + str(exc))
                    return str(exc)
            elif vol_type == 'homebase_alignment':
                # Task 3
                rospy.wait_for_service("/aligned_service", timeout=2.0)
                report_artf = rospy.ServiceProxy('/aligned_service', HomeLocationSrv)
                try:
                    result = report_artf(True)
                    print("rospy_rover: Aligned service response: %r" % result)
                    return 'ok'
                except rospy.service.ServiceException as e:
                    print("rospy_rover: Aligned service response: Incorrect")
                    return str(e)
                except rospy.ROSException as exc:
                    print("rospy_rover: /aligned_service not available: " + str(exc))
                    return str(exc)
            else:
                return ''
        else:
            return ''

    def register_handlers(self):
        QSIZE = 10

        lights_on = rospy.ServiceProxy('/' + self.robot_name + '/toggle_light', ToggleLightSrv)
        lights_on('high')

        self.light_up_pub = rospy.Publisher('/' + self.robot_name + '/sensor_controller/command', Float64, queue_size=QSIZE, latch=True)
        self.light_up_msg = Float64()

        self.brakes = rospy.ServiceProxy('/' + self.robot_name + '/brake_rover', BrakeRoverSrv)


    def run(self):
        while True:
            try:
                message = self.reqrep_socket.recv()
                response = self.process_message(message)
                self.reqrep_socket.send(response)
            except zmq.Again as e:
                pass
            if interrupted:
                self.reqrep_socket.close()
                break

class RospyRoverBase(object):
        
    def launch(self, pushpull, reqrep, argv):

        try:
            opts, args = getopt.getopt(argv, 'r:', ['robot_name=', 'push_port=', 'pull_port=', 'reqrep_port='])
        except getopt.GetoptError:
            print ("rospy_rover.py -r <robot name> --push_port<port> --pull_port=<port> --reqrep_port=<port>")
            sys.exit(2)

        for opt, arg in opts:
            if opt in ['-r', '--robot_name']:
                robot_name = arg
            elif opt in ['--push_port']:
                push_port = arg
            elif opt in ['--pull_port']:
                pull_port = arg
            elif opt in ['--reqrep_port']:
                reqrep_port = arg
            else:
                assert False, "Unhandled option"

        signal.signal(signal.SIGTERM, signal_handler) #SIGTERM - kill
        signal.signal(signal.SIGINT, signal_handler) #SIGINT - ctrl-c
        rospy.init_node('listener', anonymous=True)
        context = zmq.Context.instance()

        ppt = pushpull(robot_name, push_port, pull_port)
        ppt.setup_sockets(context)
        ppt.register_handlers()
        ppt.start()

        rrt = reqrep(robot_name, reqrep_port)
        rrt.setup_sockets(context)
        rrt.register_handlers()
        rrt.start()

        while True:
            time.sleep(100) # gives room to signal capture
            if interrupted:
                break

        ppt.join()
        rrt.join()

        context.term()

class RospyRover(RospyRoverBase):
    pass
        

# vim: expandtab sw=4 ts=4
