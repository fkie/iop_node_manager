#!/usr/bin/env python

from __future__ import division, absolute_import, print_function, unicode_literals
import rospy
import rosgraph
import ruamel.yaml
import sys
import fkie_iop_node_manager
import logging


try:
    rospy.names.reload_mappings(sys.argv)
    mappings = rospy.names.get_mappings()
    name = "iop_node_manager"
    if '__name' in mappings:
        name = mappings['__name']
    params = rospy.get_param(rosgraph.names.resolve_name(name, rospy.core.get_caller_id()), {})
    fkie_iop_node_manager.start(name, block=False, params=params)
    rospy.init_node(name)
    rospy.spin()
except Exception as err:
    import traceback
    print(traceback.format_exc())
    print("Error while initialize ROS-Node: %s" % (err), file=sys.stderr)
finally:
    try:
        fkie_iop_node_manager.shutdown()
    except Exception:
        pass
