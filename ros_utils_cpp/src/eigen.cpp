#include "ros_utils_cpp/eigen.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
// #include <Eigen/src/Core/Matrix.h>
// #include <Eigen/src/Core/VectorBlock.h>
// #include <Eigen/src/Geometry/Quaternion.h>
// #include <Eigen/src/Geometry/Transform.h>
// #include <ignition/msgs/quaternion.pb.h>
// #include <ignition/math/Pose3.hh>

namespace ros_utils_cpp
{
	namespace Eigen 
	{
		::Eigen::Affine3d 
		make_tf(const ::ignition::msgs::Vector3d& v)
		{
			::Eigen::Matrix4d T;
			T.Identity();
			T.block<3,1>(0,3) = ::Eigen::Vector3d( v.x(), v.y(), v.z() );
			return ::Eigen::Affine3d(T);
		}

		::Eigen::Affine3d
		make_tf(const ::ignition::msgs::Quaternion& q)
		{
			::Eigen::Quaternion<double> q_eig;
			q_eig.w() = q.w();
			q_eig.x() = q.x();
			q_eig.y() = q.y();
			q_eig.z() = q.z();
			return ::Eigen::Affine3d(q_eig.toRotationMatrix());
		}

		::Eigen::Affine3d 
		make_tf(const ::ignition::msgs::Pose& p)
		{
			
			::Eigen::Affine3d t = ros_utils_cpp::Eigen::make_tf(p.position());
			::Eigen::Affine3d R = ros_utils_cpp::Eigen::make_tf(p.orientation());
			::Eigen::Affine3d T = R * t;
			return T;
		}

		::Eigen::Affine3d
		make_tf(const ::ignition::math::Vector3d& v)
		{
			::Eigen::Matrix4d T;
			T.Identity();
			T.block<3,1>(0,3) = ::Eigen::Vector3d( v.X(), v.Y(), v.Z() );
			return ::Eigen::Affine3d(T);
		}

		::Eigen::Affine3d
		make_tf(const ::ignition::math::Quaterniond& q)
		{
			::Eigen::Matrix4d T;
			T.Identity();

			::Eigen::Quaternion<double> q_eig;
			q_eig.w() = q.W();
			q_eig.x() = q.X();
			q_eig.y() = q.Y();
			q_eig.z() = q.Z();

			T.block<3,3>(0,0) = q_eig.toRotationMatrix();
			
			return ::Eigen::Affine3d(T);
		}

		::Eigen::Affine3d
		make_tf(const ::ignition::math::Pose3d& p)
		{
			::Eigen::Affine3d t = ros_utils_cpp::Eigen::make_tf(p.Pos());
			::Eigen::Affine3d R = ros_utils_cpp::Eigen::make_tf(p.Rot() );
			::Eigen::Affine3d T = R * t;
			return T;
		}
	
		::Eigen::Affine3d
		make_tf(double tx, double ty, double tz,double qw, double qx, double qy, double qz)
		{
			::Eigen::Affine3d T;
			T.fromPositionOrientationScale(
				::Eigen::Vector3d(tx, ty, tz), 
				::Eigen::Quaterniond(qw, qx, qy, qz), 
				::Eigen::Vector3d(1, 1, 1));
			return T;
		}
	}
}