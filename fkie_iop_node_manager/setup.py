#!/usr/bin/env python

from distutils.core import setup

scripts=['scripts/iopnodemanager.py', 'scripts/iopparam.py', 'scripts/iopeval.py', 'scripts/rosiopnodemanager.py']
packages=['fkie_iop_node_manager', 'fkie_iop_node_manager.statistics', 'fkie_iop_node_manager.transport']
package_dir={'': 'src'}

try:
   from catkin_pkg.python_setup import generate_distutils_setup

   d = generate_distutils_setup(
      ##  don't do this unless you want a globally visible script
      scripts=scripts,
      packages=packages,
      package_dir=package_dir
   )

   setup(**d)
except ImportError:
   # install without catkin
   setup(name='IOP Node Manager',
         version='0.9.0',
         license='Apache-2.0',
         description='IOP Node Manager - Python router for IOP messages.',
         author='Alexander Tiderko',
         author_email='alexander.tiderko@fkie.fraunhofer.de',
         url='https://github.com/fkie/iop_node_manager',
         install_requires=['python-ruamel.yaml'],
         scripts=scripts,
         packages=packages,
         package_dir=package_dir
   )
