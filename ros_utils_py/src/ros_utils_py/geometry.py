#!/usr/bin/env python3

from geometry_msgs.msg import Vector3
from math import sqrt, pow, acos

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
	def angle(a: Vector3, b: Vector3) -> float:
		"""returns the angle between the two vectors a and b"""
		return acos( ( geometry.dot(a,b) ) / (  geometry.l2(a) * geometry.l2(b)) )