
#include <Eigen/Eigen>
#include <Eigen/src/Geometry/Transform.h>
#include <ignition/msgs/quaternion.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/math/Pose3.hh>

namespace ros_utils_cpp 
{
	namespace Eigen 
	{
		// convert from ::ignition::msgs::Vector3d to ::Eigen::Affine3d
		::Eigen::Affine3d 
		make_tf(const ::ignition::msgs::Vector3d& v);

		// convert from ::ignition::msgs::Quaternion to ::Eigen::Affine3d
		::Eigen::Affine3d 
		make_tf(const ::ignition::msgs::Quaternion& q);

		// convert from ::ignition::msgs::Pose to ::Eigen::Affine3d
		::Eigen::Affine3d 
		make_tf(const ::ignition::msgs::Pose& p);

		// convert from ::ignition::math::Vector3d to ::Eigen::Affine3d
		::Eigen::Affine3d
		make_tf(const ::ignition::math::Vector3d& v);

		// convert from ::ignition::math::Quaterniond to ::Eigen::Affine3d
		::Eigen::Affine3d
		make_tf(const ::ignition::math::Quaterniond& q);

		// convert from ::ignition::math::Pose3d to ::Eigen::Affine3d
		::Eigen::Affine3d
		make_tf(const ::ignition::math::Pose3d& p);

		// convert seven doubles (x, y, z, qw, qx, qy, qz) to ::Eigen::Affine3d 
		::Eigen::Affine3d
		make_tf(double tx, double ty, double tz,double qw, double qx, double qy, double qz);
	}
}
