
#include <ros/ros.h>
#include "ros_utils_cpp/utils.hpp"

int main(int argc, char **argv)
{
	// init node
	ros::init(argc, argv, "example");

	// create node handle
	ros::NodeHandle nh;

	// print using devprint
	ros_utils_cpp::utils::devprint("This is a standard message....");

	// keep this node alive
	ros_utils_cpp::utils::keep_alive(ros::this_node::getName());

	return 0;
}


