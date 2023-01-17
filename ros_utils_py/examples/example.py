#!/usr/bin/env python3

import rospy

from ros_utils_py import devprint, keep_alive

def main() -> None:
	devprint("this text is printed using the devprint configuration.... see function for more informaiton....")
	keep_alive(rospy.get_name())

if __name__ == '__main__':
	try:
		rospy.init_node("test", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
