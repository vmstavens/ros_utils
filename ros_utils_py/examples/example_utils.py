#!/usr/bin/env python3

import rospy

from ros_utils_py import devprint, keep_alive

def main() -> None:
	
	# print using devprint
	devprint("this text is printed using the devprint configuration.... see function for more informaiton....")
 
	# keep node alive
	keep_alive(rospy.get_name())


if __name__ == '__main__':
	try:
		rospy.init_node("example_utils", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
