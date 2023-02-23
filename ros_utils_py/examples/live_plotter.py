#!/usr/bin/env python3

import rospy

from random import random
import matplotlib.pyplot as plt
from ros_utils_py.plot import Plotter
from ros_utils_py.msg import PointCloud
from typing import Optional
from matplotlib.animation import FuncAnimation

def live_plotting_callback(msg: PointCloud, plotter: Plotter):
	if msg is not None:
		plotter.pcPlot(msg, "Hand pc")

# fix my creating a point cloud message from PointCloud class
# http://wiki.ros.org/ROS/Tutorials/CustomMessagePublisherSubscriber%28python%29

def main() -> None:
	# u a wizard mr.
	plotter = Plotter()
	rospy.Subscriber("live_plotter", PointCloud, live_plotting_callback, plotter)
	plt.ion()
	plt.show()
	rospy.spin()
	return

if __name__ == '__main__':
	try:
		rospy.init_node("live_plotter", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
