# cmake_minimum_required(VERSION 3.12)
# project(ros_utils_py)
# find_package(catkin REQUIRED COMPONENTS
#   rospy
# )
# catkin_python_setup()
# catkin_package()


cmake_minimum_required(VERSION 3.12)
project(ros_utils_py)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		rospy
)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
	LIBRARIES
	CATKIN_DEPENDS
		rospy
	DEPENDS
)

## process setup.py
catkin_python_setup()

## install python nodes
catkin_install_python(
  PROGRAMS 
    src/ros_utils_py/utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

