#!/usr/bin/env python

from __future__ import division, absolute_import, print_function, unicode_literals
import rospy
import rosgraph
import ruamel.yaml
import sys
import fkie_iop_node_manager


try:
    params = rospy.get_param(rosgraph.names.resolve_name("iop_node_manager", rospy.core.get_caller_id()), {})
    fkie_iop_node_manager.start("iop_node_manager", block=False, params=params)
    rospy.init_node("iop_node_manager")
    rospy.spin()
    fkie_iop_node_manager.shutdown()
except Exception as err:
    import traceback
    print(traceback.format_exc())
    print("Error while initialize ROS-Node: %s" % (err), file=sys.stderr)
