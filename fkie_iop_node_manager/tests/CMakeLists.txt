### Unit tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.

##  Python

# Unit tests not needing a running ROS core.
catkin_add_nosetests(test_addrbook.py)
catkin_add_nosetests(test_config.py)
catkin_add_nosetests(test_message.py)
catkin_add_nosetests(test_queue.py)
catkin_add_nosetests(test_server.py)
#catkin_add_nosetests(test_udp_mc.py)
#catkin_add_nosetests(test_udp_uc.py)
#catkin_add_nosetests(test_uds.py)

# Unit tests using nose, but needing a running ROS core.
find_package(rostest)
if (rostest_FOUND)
  # add_rostest(test_connect.test)
endif()