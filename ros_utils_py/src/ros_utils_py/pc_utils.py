#!/usr/bin/env python3

# from ros_utils_py import 3d

import open3d as o3d
import os
from pathlib import Path
from typing import List, Tuple, Union
import numpy as np
from numpy.typing import NDArray, ArrayLike
import random
import json
from ros_utils_py.geometry import geometry as geo
import copy
from pyntcloud import PyntCloud
import pandas as pd

class PointCloudUtils:
	class PointCloud:
		def __init__(self,*args,**kwargs):
  
			self.__COLOR_ENUM = {
				"r": ("red"   ,(1.0, 0.0, 0.0)),
				"g": ("greeb" ,(0.0, 1.0, 0.0)),
				"b": ("blue"  ,(0.0, 0.0, 1.0))
			}

			self.__input_path = kwargs.get('input_path', "")
			self.__points = kwargs.get('points', np.empty((0,3)))
			self.__normals = kwargs.get('normals', np.empty((0,3)))
			# self.__colors = kwargs.get('normals', np.empty((0,3)))
			self.__o3d_pc = kwargs.get('o3d_pc', o3d.cpu.pybind.geometry.PointCloud())

			#  array handling when extend is empty #######################################################

			if len(np.asarray(self.__o3d_pc.points)) != 0:
				self.__points = np.asarray(self.__o3d_pc.points)  
				self.__normals = np.asarray(self.__o3d_pc.normals)
				# self.__colors = np.asarray(self.__o3d_pc.colors)  
				# self.__points = np.asarray(self.__o3d_pc.points)   if len(np.asarray(self.__o3d_pc.points)) != 0 else np.array([])
				# self.__normals = np.asarray(self.__o3d_pc.normals) if len(np.asarray(self.__o3d_pc.normals)) != 0 else np.array([])
				# self.__colors = np.asarray(self.__o3d_pc.colors)   if len(np.asarray(self.__o3d_pc.colors)) != 0 else np.array([])
			# elif len(self.__points) == 0:
			# 	self.__points = np.array([])
			# 	self.__normals = np.array([])
			# 	self.__colors = np.array([])

			# empty arguments
			if "o3d_pc" not in kwargs and "input_path" not in kwargs and "points" not in kwargs:
				return

			if len(self.__points) == 0:
				return

			# in case data is provided
			self.__o3d_pc.points = o3d.utility.Vector3dVector(np.asarray(self.__points))
			# if len(self.__normals.flatten()) != 0:
			self.__o3d_pc.normals = o3d.utility.Vector3dVector(np.asarray(self.__normals))
			# if len(self.__colors.flatten()) != 0:
			# 	self.__o3d_pc.colors = o3d.utility.Vector3dVector(np.asarray(self.__colors))
			return
		
		## class methods ####################################
  
		@classmethod
		def empty(cls):
			return cls()
  
		@classmethod
		def from_file(cls,file_path: str = "", num_of_samples: int = 500, normals: bool = False):
			if file_path == "": raise ValueError("An empty string was provided to PointCloud.from_file, this is not a valid input, please provide a path.")
			if Path(file_path).suffix == ".pcd":
				if num_of_samples != 500: 
					print(f"[{__file__}] Number of samples cannot be changed from pcd file...")
				return cls(o3d_pc = o3d.io.read_point_cloud(file_path))
			mesh = o3d.io.read_triangle_mesh(file_path)
			if normals:
				mesh.compute_vertex_normals()
			pc = mesh.sample_points_uniformly(number_of_points=num_of_samples)
			return cls(o3d_pc = pc)
 
		@classmethod
		def from_data(cls, points: List[Union[List, Tuple]], normals: List[Union[List, Tuple]] = [], colors: List[Union[List, Tuple]] = []):
			pc = PointCloudUtils.PointCloud.__fill_o3d_pc(points=np.asarray(points), normals=np.asarray(normals))
			# pc = PointCloudUtils.PointCloud.__fill_o3d_pc(points=np.asarray(points), normals=np.asarray(normals), colors=np.asarray(colors))
			return cls(o3d_pc = pc)
 
		@classmethod
		def from_pc(cls,pc):
			return cls(o3d_pc=pc.o3d_pc)

		@classmethod
		def subsample(cls, pc, num_of_patches: int = 10, avg_patch_size: int = 10, timeout: int = 100):
			"""returns a subsample of the original point cloud"""

			def __patches_valid(starts: List[int], offsets: List[int]) -> bool:
				if max(offsets) + max(starts) > len(pc.points):
					print("sample out of array bound....resampling")
					return False
				is_valid = all([False if (starts[j]+offsets[j]) > (starts[j+1]) else True for j in range(len(starts)-1)])
				if is_valid:
					return is_valid
				else:
					print("patches overlap...resampling")
					return is_valid

			def __resample_patches(num_of_patches, avg_patch_size) -> Tuple[List, List]:
				temp_starts = []
				temp_offsets = []
				for i in range(num_of_patches):
					temp_starts = [random.randrange(0, len(pc.normals)) for i in range(num_of_patches)]
					temp_offsets = [avg_patch_size + random.randrange(-5, 5) for i in range(num_of_patches)]
				return temp_starts, temp_offsets

			starts, offsets = [], []
			t = 0
			while True:
				starts, offsets = __resample_patches(num_of_patches=num_of_patches, avg_patch_size=avg_patch_size)

				starts.sort()
				t += 1
				if __patches_valid(starts, offsets):
					print("found legal patches")
					break
				if timeout < t:
					print(f"[WARN]: Subsample not possible within timeout {timeout}")
					return cls(points=pc.points, normals=pc.normals, colors=pc.colors, o3d_pc=pc.o3d_pc)

			intervals = {}
			interval_file = open("/home/user/projects/shadow_robot/base/src/in_hand_pose_estimation/sr_tactile_perception/scripts/save_data/intervals.json", "w")
			sub_points, sub_normals, sub_colors = np.empty((0, 3)), np.empty((0, 3)), np.empty((0, 3))
			for i in range(len(starts)):
				intervals[f"{i}"] = [starts[i], starts[i] + offsets[i]]
				np.append(sub_points, np.array(pc.points[starts[i]:starts[i] + offsets[i]]) )
				np.append(sub_normals, np.array(pc.normals[starts[i]:starts[i] + offsets[i]]) )
				# np.append(sub_colors, np.array(pc.colors[starts[i]:starts[i] + offsets[i]]) )
				# sub_normals.extend(pc.normals[starts[i]:starts[i] + offsets[i]])
				# sub_colors.extend(pc.colors[starts[i]:starts[i] + offsets[i]])

			json.dump(intervals,interval_file,indent=4)
			interval_file.close()

			# to create a deep copy
			sub_o3d_pc = copy.deepcopy(pc.o3d_pc)

			sub_o3d_pc.points = o3d.utility.Vector3dVector(np.asarray(sub_points))
			if len(sub_normals.flatten()) != 0:
				sub_o3d_pc.normals = o3d.utility.Vector3dVector(np.asarray(sub_normals))
			# if len(sub_colors.flatten()) != 0:
			# 	sub_o3d_pc.colors = o3d.utility.Vector3dVector(np.asarray(sub_colors))
			return cls(o3d_pc=sub_o3d_pc)

		## properties ####################################

		@property
		def points(self) -> np.ndarray:
			"""the points of the point cloud"""
			return self.__points
 
		@points.setter
		def points(self, points: np.ndarray) -> None:
			if not isinstance(points, np.ndarray):
				raise TypeError(f"points must be a numpy.ndarray, not {type(points)}")
			self.__points = points
 
		@property
		def normals(self) -> np.ndarray:
			"""the normals of the point cloud"""
			return self.__normals

		@normals.setter
		def normals(self, normals: List[Union[Tuple, List]]) -> None:
			self.__normals = normals

		@property
		def colors(self) -> np.ndarray:
			"""the colors of the point cloud points"""
			return np.array([])

		@colors.setter
		def colors(self, colors: List[Union[Tuple, List]]) -> None:
			self.__colors = colors

		@property
		def o3d_pc(self) -> o3d.cpu.pybind.geometry.PointCloud:
			return self.__o3d_pc

		@property
		def shape(self) -> Tuple[int, int]:
			"""shape of the point cloud"""
			return self.points.shape

		@property
		def size(self) -> int:
			"""number of points in the point cloud"""
			return self.points.shape[0]

		@property
		def ndim(self) -> int:
			"""number of dimensions of the points"""
			return self.points.shape[1]

		def __len__(self):
			return len(self.points)

		def __getitem__(self, index):
			return self.points[index]

		def __setitem__(self, index, value):
			self.points[index] = value

		def __iter__(self):
			return iter(self.points)

		def __repr__(self):
			return f"PointCloud(points={self.points})"

		def center(self) -> np.ndarray:
			"""center point of the point cloud"""
			return np.mean(self.points, axis=0)

		def translate(self, t: np.ndarray) -> None:
			"""translate the point cloud by t"""
			self.points += t

		def scale(self, s: np.ndarray) -> None:
			"""scale the point cloud by s"""
			self.points *= s

		def rotate(self, R: np.ndarray) -> None:
			"""rotate the point cloud by R"""
			self.points = self.points @ R.T


		def write_pcd(self, filename):
		# def write_pcd(self, points, normals, filename):
			# Concatenate points and normals into a single array
			print(f"points = {len(self.__points)} | normals = {len(self.__normals)}")
			# Find the indices of points that have valid normals
			valid_indices = np.where(np.isnan(self.__normals[:,0]) == False)[0]

			# Extract the valid points and normals
			valid_points = self.__points[valid_indices - 1]
			valid_normals = self.__normals[valid_indices - 1]

			# Concatenate points and normals into a single array
			point_normals = np.concatenate((valid_points, valid_normals), axis=1)

			# Write the PointCloud to a PCD file
			with open(filename, "w") as f:
				f.write("# .PCD v0.7 - Point Cloud Data file format\n")
				f.write("VERSION 0.7\n")
				f.write("FIELDS x y z normal_x normal_y normal_z\n")
				f.write("SIZE 4 4 4 4 4 4\n")
				f.write("TYPE F F F F F F\n")
				f.write("COUNT 1 1 1 1 1 1\n")
				f.write("WIDTH {}\n".format(self.__points.shape[0]))
				f.write("HEIGHT 1\n")
				f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
				f.write("POINTS {}\n".format(self.__points.shape[0]))
				f.write("DATA ascii\n")
				np.savetxt(f, point_normals, delimiter=" ")


		def to_file(self, file_path: str, points: List = [], normals: List = [], binary: bool = False) -> None:
			self.__o3d_pc = PointCloudUtils.PointCloud.__fill_o3d_pc(points=self.__points,normals=self.__normals)
			print(f"this many normals to file {len(self.__normals)}")
			print(f"this many normals to file np {len(np.asarray(self.__o3d_pc.normals))}")
			print(f"{self.__o3d_pc.has_normals()=}")
			
			self.__o3d_pc.normals = o3d.utility.Vector3dVector(np.array(self.__normals))

			if len(points) != 0:
				pcd = o3d.t.geometry.PointCloud()
				pcd.point["positions"] = o3d.core.Tensor(points)
				pcd.point["normals"] = o3d.core.Tensor(normals)
				tmp_pc = o3d.cpu.pybind.geometry.PointCloud(pcd)
				# tmp_pc = PointCloudUtils.PointCloud(points=points, normals=normals)
				print(f"write file was successful: {o3d.io.write_point_cloud(file_path, tmp_pc, write_ascii=True)}")
				return
   
			print(f"{self.__o3d_pc.has_normals()=}")
   
			if o3d.io.write_point_cloud(file_path, self.__o3d_pc, write_ascii=True):
				print("success")
			else:
				print("sad face :(")
			return

		def extend(self, other):
			"""Extends this PointCloud with the data of another PointCloud"""
			if len(other.points) == 0:
				return PointCloudUtils.PointCloud(o3d_pc = self.o3d_pc)

			pc = o3d.cpu.pybind.geometry.PointCloud()

			self.__points = np.append( self.__points, other.points, axis=0 )
			self.__normals = np.append( self.__normals, other.normals, axis=0 )

			if len(other.points.flatten()) != 0:
				pc = PointCloudUtils.PointCloud.__fill_o3d_pc(points=self.__points,normals=self.__normals)
				# pc = PointCloudUtils.PointCloud.__fill_o3d_pc(points=points,normals=normals,colors=colors)
			else: 
				pc = o3d.cpu.pybind.geometry.PointCloud()
			# print(f"extending with {np.asarray(pc.normals)}")
			return PointCloudUtils.PointCloud(o3d_pc = pc)

		@staticmethod
		def __fill_o3d_pc(points, normals) -> o3d.cpu.pybind.geometry.PointCloud:
		# def __fill_o3d_pc(points, normals: np.ndarray = np.ndarray([]), colors: np.ndarray = np.ndarray([])) -> o3d.cpu.pybind.geometry.PointCloud:
			pc = o3d.cpu.pybind.geometry.PointCloud()
			# if len(colors.flatten()) != 0:
			# 	print(f"{colors.flatten()=}")
			# 	pc.colors = o3d.utility.Vector3dVector(np.array(colors))
			if len(points) == 0:
				raise ValueError("from_data has received an empty point cloud...")
			if len(normals) != 0:
				pc.normals = o3d.utility.Vector3dVector(np.array(normals))
			pc.points = o3d.utility.Vector3dVector(np.array(points))
			# print(f"I have filled = {np.asarray(pc.normals)}\n in normals")
			return pc

	## static utility functions ###############################################################

	@staticmethod
	def show(_pc: Union[ List[Union[str, PointCloud]] , PointCloud , str]) -> None:
		# print("[HELP] Press H while in 3D show to get keyboard shortcuts")
		if type(_pc) is PointCloudUtils.PointCloud:
			o3d.visualization.draw_geometries([_pc.o3d_pc])
			return
		
		if type(_pc) is str:
			_pc = PointCloudUtils.PointCloud.from_file(_pc)
			o3d.visualization.draw_geometries([_pc.o3d_pc])
			return
  
		l_pc = []
		for pci in _pc:
			if type(pci) is str:
				_pc = PointCloudUtils.PointCloud()
				_pc.from_file(pci)
				pci = _pc
			l_pc.append(pci.o3d_pc)
		o3d.visualization.draw_geometries([*l_pc])
		return

	@staticmethod
	def cvt_from_to(path_to_input_file: str, path_to_output_file: str, normals: bool = False, num_of_samples: int = 500) -> None:
		"""convert a file from path_to_input_file to a file in path_to_output_file. If normals = True, then normals will be estimated on the generated point cloud"""
		mesh = o3d.io.read_triangle_mesh(path_to_input_file)
		if normals:
			mesh.compute_vertex_normals()
		pc = mesh.sample_points_uniformly(number_of_points=num_of_samples)
		# Save the point cloud with normals in a PCD file format (temporary file)
		o3d.io.write_point_cloud(path_to_output_file, pc)
		return
