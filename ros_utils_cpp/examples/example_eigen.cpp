
#include "ros_utils_cpp/eigen.hpp"
#include "ros_utils_cpp/utils.hpp"
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ros/ros.h>
#include <sstream>
#include <ignition/math/Pose3.hh>

int main(int argc, char **argv)
{

	// init node
	ros::init(argc, argv, "example_eigen");

	// create node handle
	ros::NodeHandle nh;

	// string stream for printing Eigen::Affine3d
	std::stringstream ss;

	// my test ignition Vector3d
	::ignition::msgs::Vector3d v;
	v.set_x(1.0); 
	v.set_y(2.0); 
	v.set_z(3.0);

	// devprint v
	ros_utils_cpp::utils::devprint("v = \n" + v.DebugString() );

	// convert v to ::Eigen::Affine3d (4x4 transformation matrix)
	::Eigen::Affine3d T_vector = ros_utils_cpp::Eigen::make_tf(v);

	// print eigen affine matrix
	ss << T_vector.matrix();
	ros_utils_cpp::utils::devprint("T_vector = \n" + ss.str());
	ss.str("");

	// my test ignition Pose
	::ignition::msgs::Pose p;

	// devprint p
	ros_utils_cpp::utils::devprint("p = \n" + p.DebugString() );

	// convert p to ::Eigen::Affine3d (4x4 transformation matrix)
	::Eigen::Affine3d T_pose = ros_utils_cpp::Eigen::make_tf(p);

	// print eigen affine matrix
	ss << T_pose.matrix();
	ros_utils_cpp::utils::devprint("T_pose = \n" + ss.str());
	ss.str("");

	// keep this node alive
	ros_utils_cpp::utils::keep_alive(ros::this_node::getName());

}