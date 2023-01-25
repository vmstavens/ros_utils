cmake_minimum_required(VERSION 3.4)
project(ros_utils_cpp)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

################################################################################################

# dependencies

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		# ignition_msgs5
		roscpp
		roslib
		cv_bridge
		eigen_conversions
		sensor_msgs
		geometric_shapes
		gazebo_msgs
		geometry_msgs
		moveit_msgs
		moveit_core
)

# Find the Ignition msgs library
find_package(ignition-msgs5 QUIET REQUIRED)

# OpenCV
find_package(OpenCV 4 REQUIRED)

# PCL
find_package(
	PCL 1.7 REQUIRED
	COMPONENTS
		common
		io
)

add_definitions(${PCL_DEFINITIONS})

# Finding Eigen is somewhat complicated because of our need to support Ubuntu
# all the way back to saucy.  First we look for the Eigen3 cmake module
# provided by the libeigen3-dev on newer Ubuntu.  If that fails, then we
# fall-back to the version provided by cmake_modules, which is a stand-in.
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
	find_package(cmake_modules REQUIRED)
	find_package(Eigen REQUIRED)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

# Note that eigen 3.2 (on Ubuntu Wily) only provides EIGEN3_INCLUDE_DIR,
# not EIGEN3_INCLUDE_DIRS, so we have to set the latter from the former.
if(NOT EIGEN3_INCLUDE_DIRS)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

################################################################################################

# declare a catkin package

catkin_package(
	INCLUDE_DIRS
		include
		${EIGEN3_INCLUDE_DIR}
	LIBRARIES
		ros_utils_cpp
	CATKIN_DEPENDS
		roscpp
		roslib
		cv_bridge
		eigen_conversions
		sensor_msgs
		geometric_shapes
		gazebo_msgs
		geometry_msgs
	DEPENDS
		ignition-msgs5
		OpenCV
		PCL
		EIGEN3
)

################################################################################################

# include directories

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

################################################################################################

# libraries
# REMEMBER TO ADD TO INSTALL TARGETS (bottom of file) !!!

## ros_utils_cpp
add_library(ros_utils_cpp
	src/utils.cpp
	src/eigen.cpp
	src/encoding.cpp
)
target_link_libraries(
	ros_utils_cpp 
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES} 
	${PCL_LIBRARIES}
	${IGNITION-MSGS_LIBRARIES}
	)

################################################################################################

# nodes

add_executable(example_utils examples/example_utils)
target_link_libraries(example_utils ${catkin_LIBRARIES} ros_utils_cpp)

add_executable(example_eigen examples/example_eigen)
target_link_libraries(example_eigen ${catkin_LIBRARIES} ros_utils_cpp)

add_executable(example_encoding examples/example_encoding)
target_link_libraries(example_encoding ${catkin_LIBRARIES} ros_utils_cpp)

# add_executable(demo_ros_topic examples/demo_ros_topic)
# target_link_libraries(demo_ros_topic ${catkin_LIBRARIES} ros_utils)

# add_executable(demo_gazebo examples/demo_gazebo)
# target_link_libraries(demo_gazebo ${catkin_LIBRARIES} ros_utils)

# add_executable(demo_gazebo_sensors examples/demo_gazebo_sensors)
# target_link_libraries(demo_gazebo_sensors ${catkin_LIBRARIES} ros_utils)

# add_executable(demo_std examples/demo_std)
# target_link_libraries(demo_std ${catkin_LIBRARIES} ros_utils)

# add_executable(demo_geometry_msgs examples/demo_geometry_msgs)
# target_link_libraries(demo_geometry_msgs ${catkin_LIBRARIES} ros_utils)

# add_executable(demo_moveit examples/demo_moveit)
# target_link_libraries(demo_moveit ${catkin_LIBRARIES} ros_utils)

# add_executable(demo_pcl examples/demo_pcl)
# target_link_libraries(demo_pcl ${catkin_LIBRARIES} ros_utils)

################################################################################################

# install targets

## executables and/or libraries for installation
install(
	TARGETS
		ros_utils_cpp
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install header files
install(
	DIRECTORY
		include/ros_utils_cpp/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)