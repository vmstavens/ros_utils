#!/usr/bin/env python3

from geometry_msgs.msg import Vector3
from math import sqrt, pow, acos
from typing import Tuple
import tf
from math import radians
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, Pose


class geometry:
	@staticmethod
	def dot(a: Vector3, b: Vector3) -> float:
		"""perform a dot product (scalar product) of the two geometry_msgs.msg.Vector3 provided"""
		return a.x * b.x + a.y * b.y + a.z * b.z

	@staticmethod
	def pow(a: Vector3, b: float) -> Vector3:
		"""sets each element in a to the power of b, where a is a geometry_msgs.msg.Vector3 and b is a float"""
		return Vector3( pow(a.x,b), pow(a.y,b), pow(a.z,b) )

	@staticmethod
	def prod(a: Vector3, b: float) -> Vector3:
		"""multiply each element in a with b, where a is a geometry_msgs.msg.Vector3 and b is a float"""
		return Vector3( a.x*b, a.y*b, a.z*b )

	@staticmethod
	def l2(a:Vector3) -> float:
		"""returns the L2 norm of the vector e.g. euclidean distance sqrt( x² + y² + z² )"""
		return sqrt(pow(a.x, 2) + pow(a.y, 2) + pow(a.z, 2))

	@staticmethod
	def l2_dist(a: Vector3, b: Vector3):
		"""Euclidean distance between 3D points in space"""
		return sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) + pow(a.z - b.z,2) )

	@staticmethod
	def angle(a: Vector3, b: Vector3) -> float:
		"""returns the angle between the two vectors a and b"""
		return acos( ( geometry.dot(a,b) ) / (  geometry.l2(a) * geometry.l2(b)) )

	@staticmethod
	def flip(a: Vector3) -> Vector3:
		"""flips the direction of the vector a"""
		return Vector3( -1.0*a.x, -1.0*a.y, -1.0*a.z)

	@staticmethod
	def tup2vec(t: Tuple) -> Vector3:
		"""convert a tuple to a geometry.Vector3"""
		return Vector3(t[0], t[1], t[2])

	@staticmethod
	def vec2tup(v: Vector3) -> Tuple:
		"""convert a tuple to a geometry.Vector3"""
		return v.x, v.y, v.z
	
	def euler2quaternion(roll, pitch, yaw) -> Quaternion:
		"""
		Convert Euler angles to geometry quaternions.

		Parameters:
			roll (float): Roll angle in degrees.
			pitch (float): Pitch angle in degrees.
			yaw (float): Yaw angle in degrees.

		Returns:
			tuple: A tuple representing the quaternion (x, y, z, w).
		"""
		# Create a quaternion from Euler angles
		
		quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

		q = Quaternion()

		q.x = quaternion[0]
		q.y = quaternion[1]
		q.z = quaternion[2]
		q.w = quaternion[3]

		return q