#!/usr/bin/env python3

from gazebo_msgs.msg import ContactState
from typing import Dict, List, Optional, Union
from ros_utils_py.msg import PointCloudMsg
from geometry_msgs.msg import Vector3
from math import pow, acos, sqrt

class gazebo:
	@staticmethod
	def tr():
		return

# 	@staticmethod
# 	def contact_state_to_Dict(cs: Optional[ContactState]) -> Dict[str,List]:
# 		"""converts a contact state into a dictionary"""
# 		return {
# 			"contact_positions_x": [cp.x for cp in cs.contact_positions],
# 			"contact_positions_y": [cp.y for cp in cs.contact_positions],
# 			"contact_positions_z": [cp.x for cp in cs.contact_positions],
# 			"contact_normals_x"  : [cp.x for cp in cs.contact_normals],
# 			"contact_normals_y"  : [cp.y for cp in cs.contact_normals],
# 			"contact_normals_z"  : [cp.x for cp in cs.contact_normals],
# 			"forces_x"           : [cp.force.x for cp in cs.wrenches],
# 			"forces_y"           : [cp.force.y for cp in cs.wrenches],
# 			"forces_z"           : [cp.force.x for cp in cs.wrenches],
# 			"torques_x"          : [cp.torque.x for cp in cs.wrenches],
# 			"torques_y"          : [cp.torque.y for cp in cs.wrenches],
# 			"torques_z"          : [cp.torque.x for cp in cs.wrenches],
# 			"sum_forces"         : [cs.total_wrench.force.x, cs.total_wrench.force.y, cs.total_wrench.force.z],
# 			"sum_torques"        : [cs.total_wrench.torque.x, cs.total_wrench.torque.y, cs.total_wrench.torque.z],
# 			"depths"             : [d for d in cs.depths]
# 		}

# 	@staticmethod
# 	def point_cloud_to_dict(pc: PointCloud) -> Dict[str,Union[List,bool]]:
# 		"""converts a point cloud from ros_utils_py into a dictionary"""
# 		return {
# 			"positions_x": [p.x for p in pc.positions],
# 			"positions_y": [p.y for p in pc.positions],
# 			"positions_z": [p.z for p in pc.positions],
# 			"normals_x"  : [n.x for n in pc.normals],
# 			"normals_y"  : [n.y for n in pc.normals],
# 			"normals_z"  : [n.z for n in pc.normals],
# 			"empty"      : (len(pc.positions) == 0)
# 		}

# 	@staticmethod
# 	def combine_point_clouds(*args: PointCloudMsg) -> PointCloudMsg:
# 		"""takes a varying number of PointCloudMsg types and combine them into one point cloud"""

# 		res = PointCloudMsg()
# 		is_empty: List[bool] = []
# 		# args is a tuple of pc arguments
# 		for point_cloud in args:
# 			if point_cloud.positions is None or point_cloud.normals is None:
# 				raise ValueError("Position value of type None cannot be combined in combine_point_clouds")

# 			res.positions.extend([p for p in point_cloud.positions])
# 			res.normals.extend([n for n in point_cloud.normals])
# 			res.colors.extend([c for c in point_cloud.colors])
# 			is_empty.append(point_cloud.empty)
# 		res.empty = True if all(is_empty) else False
# 		return res