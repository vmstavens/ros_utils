
#include <Eigen/Eigen>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/msgs/pose.pb.h>

namespace ros_utils_cpp 
{
	namespace Eigen 
	{
		// convert from ::ignition::msgs::Vector3d to ::Eigen::Affine3d
		::Eigen::Affine3d 
		toEigen(const ::ignition::msgs::Vector3d& v);

		// convert from ::ignition::msgs::Pose to ::Eigen::Affine3d
		::Eigen::Affine3d 
		toEigen(const ::ignition::msgs::Pose& p);
	}
}
