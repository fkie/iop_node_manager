cmake_minimum_required(VERSION 3.0.2)
project(fkie_iop_node_manager)

find_package(catkin REQUIRED COMPONENTS rospy)

###################################
## catkin specific configuration ##
###################################
catkin_python_setup()
catkin_package(
)

catkin_install_python(
    PROGRAMS 
        scripts/iopeval.py
        scripts/iopnodemanager.py
        scripts/iopparam.py
        scripts/rosiopnodemanager.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Add folders to be run by python nosetests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
