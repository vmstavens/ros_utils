#include "ros_utils_cpp/eigen.hpp"

namespace ros_utils_cpp
{
	namespace Eigen 
	{
		::Eigen::Affine3d 
		toEigen(const ::ignition::msgs::Vector3d& v)
		{
			::Eigen::Matrix4d T;
			T.Identity();
			T.block<3,1>(0,3) = ::Eigen::Vector3d( v.x(), v.y(), v.z() );
			return ::Eigen::Affine3d(T);
		}

		::Eigen::Affine3d 
		toEigen(const ::ignition::msgs::Pose& p)
		{
			::Eigen::Quaternion<float> q(
				p.orientation().w(),
				p.orientation().x(),
				p.orientation().y(),
				p.orientation().z());
			
			::Eigen::Vector3d t(
				p.position().x(),
				p.position().y(),
				p.position().z());
	
			::Eigen::Matrix3d R = q.toRotationMatrix().cast<double>();

			::Eigen::Matrix4d T;
			T.setIdentity();
			T.block<3,3>(0,0) = R;
			T.block<3,1>(0,3) = t;

			return ::Eigen::Affine3d(T);
		}
	}
}