#!/usr/bin/env python

from __future__ import division, absolute_import, print_function, unicode_literals
import rospy
import fkie_iop_node_manager


fkie_iop_node_manager.start("iop_node_manager", block=False)
rospy.init_node("iop_node_manager")
rospy.spin()
fkie_iop_node_manager.shutdown()
