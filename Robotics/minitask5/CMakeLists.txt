cmake_minimum_required(VERSION 2.8.3)
project(minitask5)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs)

catkin_install_python(PROGRAMS src/navigation.py src/main.py src/bresenhamline.py src/callbacks.py src/gridmap.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
