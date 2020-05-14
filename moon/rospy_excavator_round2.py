"""
  Proxy for ROS sensors and effectors
  this is Python 2.7 code
"""

import sys
import rospy
import yaml

from rospy_excavator import RospyExcavator, RospyExcavatorReqRep, RospyExcavatorPushPull
from srcp2_msgs.srv import Qual2VolatilesSrv

class RospyExcavatorRound2ReqRep(RospyExcavatorReqRep):
    def __init__(self, robot_name, reqrep_port):
        super(RospyExcavatorRound2ReqRep, self).__init__(robot_name, reqrep_port)

    def register_handlers(self):
        QSIZE = 10

        self.get_volatile_list = rospy.ServiceProxy('/qual_2_services/volatile_locations', Qual2VolatilesSrv)

    def process_message(self, message):
#        print("OSGAR:" + message)
        result = super(RospyExcavatorRound2ReqRep, self).process_message(message)
        if len(result) == 0:
            message_type = message.split(" ")[0]
            if message_type == "get_volatile_locations":
                result = self.get_volatile_list()
                vol_list = yaml.safe_load(str(result))
                return ' '.join((str(e['x']) + " " + str(e['y'])) for e in vol_list['poses'])
            else:
                return ''

class RospyExcavatorRound2(RospyExcavator):
    pass        
        
if __name__ == '__main__':
    re = RospyExcavatorRound2()
    re.launch(RospyExcavatorPushPull, RospyExcavatorRound2ReqRep, sys.argv[1:])

# vim: expandtab sw=4 ts=4
