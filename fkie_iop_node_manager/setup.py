from setuptools import setup

package_name = 'fkie_iop_node_manager'

setup(
   name=package_name,
   version='0.9.0',
   packages=[package_name, package_name + '.statistics', package_name + '.transport'],
   data_files=[
         ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
#         ('share/' + package_name + '/launch', ['launch/autostart.launch.xml']),
         ('share/' + package_name, ['package.xml']),
      ],
   install_requires=['setuptools', 'ruamel.yaml'],
   zip_safe=True,
   maintainer='Alexander Tiderko',
   maintainer_email='Alexander.Tiderko@fkie.fraunhofer.de',
   description='A daemon node to manage ROS launch files and launch nodes from loaded files.',
   license='Apache License, Version 2.0',
   url='https://github.com/fkie/ros_node_manager',
   tests_require=['pytest'],
   test_suite="tests",
   entry_points={
      'console_scripts': [
         'rosiopnodemanager.py ='
         ' fkie_iop_node_manager:main',
      ],
   },
   scripts = [
      'scripts/iopeval.py',
      'scripts/iopnodemanager.py',
      'scripts/iopparam.py'
   ]
)
