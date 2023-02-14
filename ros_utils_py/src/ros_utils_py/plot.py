
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.gridspec import GridSpec
from matplotlib import pyplot as plt
import numpy as np
from typing import List, Dict
from ros_utils_py.utils import devprint
matplotlib.use("GTK3Agg")

class Plotter:
	def __init__(self) -> None:
		self.fig_2D = plt.figure("2D figures")
		self.num_of_2D_plots = 1
		self.ax_2D = self.fig_2D.add_subplot(self.num_of_2D_plots, 1, 1)
		self.plot_data_2D: Dict[str,List] = {}
		self.scatter_data_2D: Dict[str,Dict] = {}
  
		self.fig_3D = plt.figure("3D figures")
		self.num_of_3D_plots = 1
		self.plot_data_3D: Dict[str,Dict] = {}
		self.scatter_data_3D: Dict[str,Dict] = {}
		self.ax_3D = self.fig_3D.add_subplot(self.num_of_2D_plots, 1, 1, projection="3d")
  
		self.fig_2D.tight_layout()
		self.fig_3D.tight_layout()
  
		plt.show(block=False)

	def _update(self):
		plt.clf()

		fig_counter_2D = 1

		# 2D Plot
		for key in self.plot_data_2D.keys():
			self.ax_2D = self.fig_2D.add_subplot(self.num_of_2D_plots, 1, fig_counter_2D)
			name = key
			data = self.plot_data_2D[name]
			plt.title(name)
			plt.plot(data)
			fig_counter_2D += 1

		devprint(f"{fig_counter_2D=}")

		# 2D Scatter
		for key in self.scatter_data_2D.keys():
			self.ax_2D = self.fig_2D.add_subplot(self.num_of_2D_plots, 1, fig_counter_2D)
			name = key
			data = self.scatter_data_2D[name]
			plt.title(name)
			plt.scatter(data["x"],data["y"])
			fig_counter_2D += 1

		fig_counter_3D = 1
		# 3D Scatter
		for key in self.scatter_data_3D.keys():
			self.ax_3D = self.fig_3D.add_subplot(self.num_of_3D_plots, 1, fig_counter_3D,  projection="3d")
			name = key
			data = self.scatter_data_3D[name]
			plt.title(name)
			# coordinate system limits
			self.ax_3D.set_xlim(0, 1)
			self.ax_3D.set_ylim(0, 1)
			self.ax_3D.set_zlim(0, 1)
			self.ax_3D.scatter(data["x"], data["y"], data["z"])
			fig_counter_3D += 1

		plt.draw()
		plt.pause(0.00000000001)

	def plot2D(self, name :str, x: List) -> None:
		# if the name given if not registered, append it to the data list
		if name in self.plot_data_2D:
			# update the entry's data
			self.plot_data_2D[name] = x
		else:
			# add data entry to dict
			self.plot_data_2D[name] = x
			# update num of plots
			self.num_of_2D_plots = len(self.scatter_data_2D.keys()) + len(self.plot_data_2D.keys())
		# update plots
		self._update()
		
	def scatter2D(self, name: str, x: List, y: List) -> None:
		# if the name given if not registered, append it to the data list
		if name in self.scatter_data_2D:
			# update the entry's data
			self.scatter_data_2D[name] = {"x":x, "y":y}
		else:
			# add data entry to dict
			self.scatter_data_2D[name] = {"x": x, "y": y}
			# update num of scatters
			self.num_of_2D_plots = len(self.scatter_data_2D.keys()) + len(self.plot_data_2D.keys())
		# update plots
		self._update()

	def scatter3D(self, name: str, x: List, y: List, z: List) -> None:
		# if the name given if not registered, append it to the data list
		if name in self.scatter_data_3D:
			# normalize data between 0 and 1
			x_norm = [float(i)/sum(x) for i in x]
			y_norm = [float(i)/sum(y) for i in y]
			z_norm = [float(i)/sum(z) for i in z]
			# update the entry's data
			self.scatter_data_3D[name] = {"x": x_norm, "y": y_norm, "z": z_norm}
			# self.scatter_data_3D[name] = {"x": x, "y": y, "z": z}
		else:
			# add data entry to dict
			self.scatter_data_3D[name] = {"x": x, "y": y, "z": z}
			# update num of scatters
			self.num_of_3D_plots = len(self.scatter_data_3D.keys()) + len(self.plot_data_3D.keys())
		# update plots
		self._update()
