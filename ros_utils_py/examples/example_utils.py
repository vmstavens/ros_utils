#!/usr/bin/env python3

import rospy

from ros_utils_py import devprint, keep_alive

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Vector3, Twist


def gms_client(model_name, relative_entity_name):
	rospy.wait_for_service('/gazebo/get_model_state')
	try:
		gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		resp1 = gms(model_name, relative_entity_name)
		return resp1
	except rospy.ServiceException:
		return
		# print("Service call failed: %s" % e)

def attempt1():
	# Create a publisher to publish the twist message
	twist_pub = rospy.Publisher('/cube/cmd_vel', Twist, queue_size=1)
	# Create a twist message to change the velocity of the model
	twist_msg = Twist()
	twist_msg.linear.x = 0.1   # set linear velocity in x-direction


def main() -> None:
	
	# print using devprint
	devprint("this text is printed using the devprint configuration.... see function for more information....")
 
 
	attempt1()
 
 
	# model_name = "cube"
	# relative_entity_name = ""
 
	# res = gms_client(model_name, relative_entity_name)
 
	# pub = rospy.Publisher("gazebo/set_model_state",ModelState)
	# ms = ModelState()
	# ms.twist.linear = Vector3(*[0.0, 0.5, 0.0])
 
	# keep node alive
	keep_alive(rospy.get_name())


if __name__ == '__main__':
	try:
		rospy.init_node("example_utils", anonymous=True)
		main()
		
	except rospy.ROSInterruptException:
		pass


# rostopic pub - 1 / gazebo/set_model_state gazebo_msgs/ModelState “model_name: ‘table’
# pose:
#   position:
#     x: 0.0
#     y: 0.0
#     z: 0.0
# “
