#!/usr/bin/env python3

import rospy
from random import random
from std_msgs.msg import Float32MultiArray

def main() -> None:
	publisher = rospy.Publisher("example_plot_pub",Float32MultiArray,queue_size=1)

	# publish a float between 0 and 1 every 0.5 second
	while not rospy.is_shutdown():
		msg = Float32MultiArray()
		msg.data = [random() for i in range(10)]
		publisher.publish( msg )
		rospy.sleep(0.5)

if __name__ == "__main__":
	try:
		rospy.init_node("example_plot_pub", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
