#!/usr/bin/env python

from __future__ import division, absolute_import, print_function, unicode_literals
import rospy
import rosgraph
import sys
import fkie_iop_node_manager


def get_ros_loglevel(loglevel):
    if loglevel == 'info':
        return rospy.INFO
    elif loglevel == 'debug':
        return rospy.DEBUG
    elif loglevel == 'warning':
        return rospy.WARN
    elif loglevel == 'error':
        return rospy.ERROR
    elif loglevel == 'critical':
        return rospy.FATAL


try:
    rospy.names.reload_mappings(sys.argv)
    mappings = rospy.names.get_mappings()
    name = "iop_node_manager"
    if '__name' in mappings:
        name = mappings['__name']
    params = rospy.get_param(rosgraph.names.resolve_name(name, rospy.core.get_caller_id()), {})
    fkie_iop_node_manager.start(name, block=False, params=params)
    rospy.init_node(name, log_level=get_ros_loglevel(fkie_iop_node_manager.server.loglevel))
    rospy.spin()
except Exception as err:
    import traceback
    print(traceback.format_exc())
    print("Error while initialize ROS-Node: %s" % (err), file=sys.stderr)
finally:
    fkie_iop_node_manager.shutdown()
