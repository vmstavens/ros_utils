#!/usr/bin/env python3

import rospy

from random import random
import matplotlib.pyplot as plt
from ros_utils_py.plot import Plotter
from std_msgs.msg import Float32MultiArray


def live_plotting_callback(msg: Float32MultiArray, plotter: Plotter):
	if msg.data != None:
		plotter.plot2D("name1", msg.data)
		plotter.plot2D("name2", [random() for i in range(10)])
		plotter.scatter2D("name3", [random() for i in range(10)], [random() for i in range(10)])
		plotter.scatter3D("name2",
			[random() for i in range(10)], 
			[random() for i in range(10)],
			[random() for i in range(10)])

def main() -> None:
	# u a wizard mr. 
	plotter = Plotter()
	rospy.Subscriber("example_plot_pub", Float32MultiArray, live_plotting_callback, plotter)

	plt.ion()
	plt.show()
	rospy.spin()
	return

if __name__ == '__main__':
	try:
		rospy.init_node("example_plot", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
