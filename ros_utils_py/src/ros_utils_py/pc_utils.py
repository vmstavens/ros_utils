#!/usr/bin/env python3

# from ros_utils_py import 3d

import open3d as o3d
import os
from pathlib import Path
from typing import List, Tuple, Union
import numpy as np
from numpy.typing import NDArray
import random


class PointCloudUtils:
	
	class PointCloud:
		def __init__(self, 
               points: List[Union[List, Tuple]] = [], 
               normals: List[Union[List, Tuple]] = [], 
               colors: List[Union[List, Tuple]] = [], 
               input_path: str = ""):  # type: ignore

			# in case the pc is read from a file
			if input_path != "":
				__pc = self.__from_file(input_path)
				# colors are defined as list of tuples [(r,g,b), (r,g,b), (r,g,b)] where r, g and b are [0,1]
				self.__points, self.__normals, self.__colors, self.__o3d_pc = self.__extract_members(__pc)
				return

			if self.__members_are_empty(points, normals, colors):
			# if self.__members_are_empty(points, normals, colors, intensities):
				return 

			# in case data is provided
			self.__points = o3d.utility.Vector3dVector(np.asarray(points))
			self.__normals = o3d.utility.Vector3dVector(np.asarray(normals))
			self.__colors = o3d.utility.Vector3dVector(np.asarray(colors))
			return
		
		@property
		def points(self) -> NDArray:
			"""the points of the point cloud"""
			return self.__points
 
		@property
		def normals(self) -> NDArray:
			"""the normals of the point cloud"""
			return self.__normals

		@property
		def colors(self) -> NDArray:
			"""the colors of the point cloud points"""
			return self.__colors

		def subsample(self, num_of_patches: int = 10, avg_patch_size: int = 10):
			"""returns a subsample of the original point cloud"""
			# TODO : FIX DIS SHIT
			starts = [random.randrange(0, len(self.__normals)) for i in range(num_of_patches)]
			offset = avg_patch_size + random.randrange(-5,5 + 1) # the last is not included
			points = [self.__points[starts[i]:(starts[i] + avg_patch_size)] for i in range(len(starts))]
			a = PointCloudUtils.PointCloud(points=self.__points)
			return PointCloudUtils.PointCloud()

		def from_file(self, file_path, normals: bool = False, num_of_samples: int = 500) -> None:
			__pc = self.__from_file(file_path=file_path, normals=normals,num_of_samples=num_of_samples)
			self.__points, self.__normals, self.__colors, self.__o3d_pc = self.__extract_members(__pc)
			return

		def to_file(self, file_path: str) -> None:
			o3d.io.write_point_cloud(file_path, self.__o3d_pc)
			return

		def __extract_members(self, pc: o3d.cpu.pybind.geometry.PointCloud):
			""""simply extracts the members from the point cloud into a tuple of NDArrays from Numpy"""
			return (np.asarray(pc.points), np.asarray(pc.normals), np.asarray(pc.colors), pc)
			# return (np.asarray(pc.points), np.asarray(pc.normals), np.asarray(pc.intensities), np.asarray(pc.colors), pc)

		def __members_are_empty(self, points: List, normals: List, colors: List) -> bool:
			return (len(points) == 0 and len(normals) == 0 and len(colors) == 0)

		def __from_file(self, file_path, normals: bool = False, num_of_samples: int = 500) -> o3d.cpu.pybind.geometry.PointCloud:
			if Path(file_path).suffix == ".pcd":
				if num_of_samples != 500: print(f"[{__file__}] Number of samples cannot be changed from pcd file...")
				return o3d.io.read_point_cloud(file_path)
			mesh = o3d.io.read_triangle_mesh(file_path)
			if normals:
				mesh.compute_vertex_normals()
			pc = mesh.sample_points_uniformly(number_of_points=num_of_samples)
			return pc
 

	@staticmethod
	def show(pc: List[Union[str, PointCloud]]) -> None:
		l_pc = []
		for pci in pc:
			if type(pci) is str:
				_pc = PointCloudUtils.PointCloud()
				_pc.from_file(pci)
				pci = _pc
			l_pc.append(PointCloudUtils.__make_o3d_pc(pci))
		print(l_pc)
		o3d.visualization.draw_geometries([*l_pc])
		print("[HELP] Press H while in 3D show to get keyboard shortcuts")
		return
 
	@staticmethod
	def __make_o3d_pc(pc: PointCloud) -> o3d.cpu.pybind.geometry.PointCloud:
		# create an empty point cloud
		pcd: o3d.cpu.pybind.geometry.PointCloud = o3d.cpu.pybind.geometry.PointCloud()
		# add attributes to point cloud depending on input
		if len(pc.normals) != 0:
			pcd.normals = o3d.utility.Vector3dVector(np.array(pc.normals))
		if len(pc.colors) != 0:
			pcd.colors = o3d.utility.Vector3dVector(np.array(pc.colors))
		if len(pc.points) == 0:
			raise ValueError("data2pc in ros_utils_py.o3d has received an empty point cloud...")
		pcd.points = o3d.utility.Vector3dVector(np.array(pc.points))
		return pcd

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