#!/usr/bin/python
"""
  Proxy for ROS sensors and effectors
  this is Python 2.7 code
"""

import sys
import rospy

from rospy_rover import RospyRover, RospyRoverReqRep, RospyRoverPushPull

class RospyHaulerPushPull(RospyRoverPushPull):
    def __init__(self, robot_name, push_port, pull_port):
        super(RospyHaulerPushPull, self).__init__(robot_name, push_port, pull_port)
        
    def register_handlers(self):
        super(RospyHaulerPushPull, self).register_handlers()

class RospyHaulerReqRep(RospyRoverReqRep):
    def __init__(self, robot_name, reqrep_port):
        super(RospyHaulerReqRep, self).__init__(robot_name, reqrep_port)


class RospyHauler(RospyRover):
        
    def launch(self, argv):
        super(RospyHauler, self).launch(RospyHaulerPushPull, RospyHaulerReqRep, argv)
        
if __name__ == '__main__':
    rh = RospyHauler()
    rh.launch(sys.argv[1:])

# vim: expandtab sw=4 ts=4
