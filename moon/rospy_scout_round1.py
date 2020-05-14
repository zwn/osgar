"""
  Proxy for ROS sensors and effectors
  this is Python 2.7 code

  scout enhanced with reporting detection of volatiles (task specific to Round 1 of Qualification)
"""

import sys
import rospy

from rospy_scout import RospyScout, RospyScoutReqRep, RospyScoutPushPull
from srcp2_msgs.msg import Qual1ScoringMsg
from srcp2_msgs.srv import Qual1ScoreSrv


class RospyScoutRound1PushPull(RospyScoutPushPull):
    pass

class RospyScoutRound1ReqRep(RospyScoutReqRep):
    def __init__(self, robot_name, reqrep_port):
        super(RospyScoutRound1ReqRep, self).__init__(robot_name, reqrep_port)

    def process_message(self, message):
        result = super(RospyScoutRound1ReqRep, self).process_message(message)
        if len(result) == 0:
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
        else:
            return result

class RospyScoutRound1(RospyScout):
    pass
        
if __name__ == '__main__':
    rs = RospyScoutRound1()
    rs.launch(RospyScoutRound1PushPull, RospyScoutRound1ReqRep, sys.argv[1:])
