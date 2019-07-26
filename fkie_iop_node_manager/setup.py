#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   ##  don't do this unless you want a globally visible script
   scripts=['scripts/iopnodemanager.py', 'scripts/iopparam.py', 'scripts/iopeval.py', 'scripts/rosiopnodemanager.py'],
   packages=['fkie_iop_node_manager', 'fkie_iop_node_manager.statistics', 'fkie_iop_node_manager.transport'],
   package_dir={'': 'src'}
)

setup(**d)