
#include "ros_utils_cpp/encoding.hpp"
#include "ros_utils_cpp/utils.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{

	// init node
	ros::init(argc, argv, "example_encoding");

	// create node handle
	ros::NodeHandle nh;

	// some data
	std::string myData = "Well, hello there";
	ros_utils_cpp::utils::devprint("my data: " + myData);

	// encode the data
	std::string encodedData = ros_utils_cpp::encoding::base64_encode(myData);
	ros_utils_cpp::utils::devprint("base 64 encoded data: " + encodedData);

	// decode the data
	std::string decodedData = ros_utils_cpp::encoding::base64_decode(encodedData);
	ros_utils_cpp::utils::devprint("base 64 decoded data: " + decodedData);

	// keep this node alive
	ros_utils_cpp::utils::keep_alive(ros::this_node::getName());

}