"""
  Proxy for ROS sensors and effectors
  this is Python 2.7 code
"""

import sys
import rospy

from std_msgs.msg import *  # Float64, JointState
from sensor_msgs.msg import *
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point

from rospy_rover import RospyRover, RospyRoverReqRep, RospyRoverPushPull
from srcp2_msgs.msg import VolSensorMsg, Qual1ScoringMsg
from srcp2_msgs.srv import Qual1ScoreSrv


class RospyScoutPushPull(RospyRoverPushPull):
    def __init__(self, robot_name, push_port, pull_port):
        super(RospyScoutPushPull, self).__init__(robot_name, push_port, pull_port)
        
    def register_handlers(self):
        super(RospyScoutPushPull, self).register_handlers()

        rospy.Subscriber('/' + self.robot_name + '/volatile_sensor', VolSensorMsg, self.callback_topic, '/' + self.robot_name + '/volatile_sensor')

class RospyScoutReqRep(RospyRoverReqRep):
    def __init__(self, robot_name, reqrep_port):
        super(RospyScoutReqRep, self).__init__(robot_name, reqrep_port)

    def process_message(self, message):
#        print("OSGAR:" + message)
        message_type = message.split(" ")[0]
        if message_type == "artf":
            s = message.split()[1:]  # ignore "artf" prefix
            vol_type = s[0]

            if vol_type in ['ice', 'ethene', 'methane', 'methanol', 'carbon_dio', 'ammonia', 'hydrogen_sul', 'sulfur_dio']:
                # Task 1
                x, y, z = [float(a) for a in s[1:]]
                pose = geometry_msgs.msg.Point(x, y, z)
                print ("rospy_scout: Reporting %s at position %f %f %f" % (vol_type, x, y, z))
                try:
                    rospy.wait_for_service("/vol_detected_service", timeout=2.0)
                    report_artf = rospy.ServiceProxy('/vol_detected_service', Qual1ScoreSrv)
                    resp = report_artf(pose=pose, vol_type=vol_type)
                    print ("rospy_scout: Volatile report result: %r" % resp.result)
                    return 'ok'
                except rospy.ServiceException as exc:
                    print("rospy_scout: /vol_detected_service exception: " + str(exc))
                    return str(exc)
                except rospy.ROSException as exc:
                    print("rospy_scout: /vol_detected_service not available: " + str(exc))
                    return str(exc)
            else:
                return ''
                    
        else:
            return ''


class RospyScout(RospyRover):
        
    def launch(self, argv):
        super(RospyScout, self).launch(RospyScoutPushPull, RospyScoutReqRep, argv)
        
if __name__ == '__main__':
    rs = RospyScout()
    rs.launch(sys.argv[1:])
