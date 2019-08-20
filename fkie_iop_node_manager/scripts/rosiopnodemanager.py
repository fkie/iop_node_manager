#!/usr/bin/env python

from __future__ import division, absolute_import, print_function, unicode_literals
import rospy
import sys
import fkie_iop_node_manager


fkie_iop_node_manager.start("iop_node_manager", block=False)
try:
    rospy.init_node("iop_node_manager")
    rospy.spin()
except Exception as err:
    print("Error while initialize ROS-Node: %s" % (err), file=sys.stderr)
fkie_iop_node_manager.shutdown()
