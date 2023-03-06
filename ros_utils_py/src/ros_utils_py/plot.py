
from typing import Dict, List, Tuple

import matplotlib
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D, proj3d
from ros_utils_py.msg import PointCloud, Color
from ros_utils_py.log import Logger
from ros_utils_py.utils import COLORS_RGBA
import rospy
from std_msgs.msg import ColorRGBA
import numpy as np

matplotlib.use("GTK3Agg")

class Plotter:

	class Arrow3D(FancyArrowPatch):
		def __init__(self, start: Vector3, end: Vector3, length: float = 1.0, *args, **kwargs):
			FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
			x = (start.x, end.x)
			y = (start.y, end.y)
			z = (start.z, end.z)
			
			self._verts3d = x, y, z

		def draw(self, renderer):
			# Tuple[float, float, float]     Tuple[float, float, float]      Tuple[float, float, float]
			xs3d, ys3d, zs3d = self._verts3d
			xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
			self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
			FancyArrowPatch.draw(self, renderer)

	def __init__(self) -> None:
		
		self.__log = Logger()
  
		self.fig_3D = plt.figure("3D figures")
		self.num_of_3D_plots = 1
		self.ax_3D = self.fig_3D.add_subplot(1, 1, 1, projection="3d")

		self.fig_3D.tight_layout()
  
		self.__zero_pc:         PointCloud = PointCloud()
		self.__prev_pc:         PointCloud = self.__zero_pc
		self.__curr_pc:         PointCloud = self.__zero_pc
		self.DEFAULT_CENTROID:  Vector3 = Vector3(0.5, 0.5, 0.5)
		self.centroid3D:        Vector3 = Vector3(0.5, 0.5, 0.5)
		self.centroid3D_pad:    float = 0.1
		self.n_length:          float = self.centroid3D_pad * 0.3
		# a: float_32 = 1.0
		# in case of weird error with illegal access to private variables...
		while True:
			try:
				self.__log.warn("Trying to enable live plotter...")
				plt.show(block=False)
				self.__log.success("Succeeded to enable live plotter...")
				break
			except:
				self.__log.error("Failed to enable live plotter... trying again...")
				continue

	def find_uiques(self,l: List[Color]):
		unique_list = []
		for c in l:
			# does c exist in unique_list
			for u in unique_list:
				if u != c:
					continue
				else:
					break
			unique_list.append(c)
		return

	def pcPlot(self, pc: PointCloud, title: str = "Title") -> None:

		plt.clf()
		plt.title(title)

		unique_colors: List[str] = list(set([ c.label for c in pc.colors ]))
	
		# if no data is received, just plot the previously received data
		self.__prev_pc = self.__curr_pc
		self.__curr_pc = pc if not pc.empty else self.__prev_pc

		self.ax_3D = self.fig_3D.add_subplot(1, 1, 1, projection="3d")
  
		legend_labels: Dict[str,str] = {
				COLORS_RGBA.MAGENTA.label: "thumb_finger",
				COLORS_RGBA.BLUE.label   : "index_finger",
				COLORS_RGBA.GREEN.label  : "middle_finger",
				COLORS_RGBA.YELLOW.label : "ring_finger",
				COLORS_RGBA.RED.label    : "pinky_finger"
		}

		for l in unique_colors:

			# get the indices of the color c
			colors_indices = [i for i, ci in enumerate(pc.colors) if ci.label == l]

			# cant do a tuple look up?! good luck ------------------------------------------------------------------
			legend: str = legend_labels[l]
   
			# get the position points
			points_xs = [pc.positions[index_c].x for index_c in colors_indices]
			points_ys = [pc.positions[index_c].y for index_c in colors_indices]
			points_zs = [pc.positions[index_c].z for index_c in colors_indices]

			# get the normals
			normals_xs = [pc.normals[index_c].x for index_c in colors_indices]
			normals_ys = [pc.normals[index_c].y for index_c in colors_indices]
			normals_zs = [pc.normals[index_c].z for index_c in colors_indices]
  
			# just some standard value for the normalized centroid to be overwritten
			self.centroid3D = Vector3(0.0, 0.0, 0.0)

			# compute the normalized centroid, if the data is empty, set to default centroid
			if pc.empty:
				self.centroid3D = self.DEFAULT_CENTROID
			else:
				self.centroid3D = Vector3(sum(points_xs) / len(points_xs), sum(points_ys) / len(points_ys), sum(points_zs) / len(points_zs))

			# limit the axis for constant reference
			self.ax_3D.set_xlim(self.centroid3D.x - 0.5 * self.centroid3D_pad, self.centroid3D.x + 0.5 * self.centroid3D_pad)
			self.ax_3D.set_ylim(self.centroid3D.y - 0.5 * self.centroid3D_pad, self.centroid3D.y + 0.5 * self.centroid3D_pad)
			self.ax_3D.set_zlim(self.centroid3D.z - 0.5 * self.centroid3D_pad, self.centroid3D.z + 0.5 * self.centroid3D_pad)
   
			self.ax_3D.set_xlabel("x [m]")
			self.ax_3D.set_ylabel("y [m]")
			self.ax_3D.set_zlabel("z [m]")

			# plot the point cloud
			color_code = pc.colors[colors_indices[0]].color_code
			self.ax_3D.scatter(points_xs, points_ys, points_zs, c = color_code, label = legend)
			self.ax_3D.legend()
   
			# iterate over all points
			for i in range(len( points_xs )):

				start  = Vector3(points_xs[i], points_ys[i], points_zs[i])
				normal = Vector3(normals_xs[i] * self.n_length, normals_ys[i] * self.n_length, normals_zs[i] * self.n_length)
				end    = Vector3(start.x + normal.x, start.y + normal.y, start.z + normal.z)

				# draw normals
				n = self.Arrow3D(
					start   = start,
					end     = end,
					mutation_scale = 5, lw=1, arrowstyle = "-|>", color = color_code) # this is just the style
				
				self.ax_3D.add_artist(n)

			# self.__log.warn(f"Currently I have {len(unique_colors)} unique colors...")
			# if len(unique_colors) == 4:
			# 	self.__log.success(f"Saved figure ------------------------------")
			# 	plt.savefig("/home/user/projects/shadow_robot/base/src/in_hand_pose_estimation/sr_tactile_perception/plot3d.png")

		# plt.draw()
		self.fig_3D.canvas.draw_idle()
		self.fig_3D.canvas.flush_events()
		plt.pause(0.00000000001)