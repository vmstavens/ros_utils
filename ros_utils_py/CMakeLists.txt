cmake_minimum_required(VERSION 3.4)
# cmake_minimum_required(VERSION 3.12)
project(ros_utils_py)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		rospy
		std_msgs
		geometry_msgs
		gazebo_msgs
		message_generation
		message_runtime
)

add_message_files(FILES Color.msg PointCloudMsg.msg)

## process setup.py
catkin_python_setup()

generate_messages(
	DEPENDENCIES
		std_msgs
		gazebo_msgs
		geometry_msgs
)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
	LIBRARIES
	CATKIN_DEPENDS
		rospy
		std_msgs
		gazebo_msgs
		geometry_msgs
		message_generation
		message_runtime
	DEPENDS
)

## install python nodes
catkin_install_python(
  PROGRAMS 
    src/ros_utils_py/utils.py
    examples/example_utils.py
    examples/example_plot.py
    examples/example_plot_pub.py
    examples/live_plotter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

