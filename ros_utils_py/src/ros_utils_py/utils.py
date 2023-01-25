#!/usr/bin/env python3

from datetime import datetime
import time
import signal

def devprint(msg: str) -> None:
	"""prints the input string in an easy to spot format when scrolling through the ros gazebo cluttered terminal. The default formatting is blue and bold text, but can be configured as pleased.

	Args:
		msg (str): the string to be printed
	"""
	class COLORS:
		PURPLE    = '\033[95m'
		CYAN      = '\033[96m'
		DARKCYAN  = '\033[36m'
		BLUE      = '\033[94m'
		GREEN     = '\033[92m'
		YELLOW    = '\033[93m'
		RED       = '\033[91m'
		NONE      = ''

	class TXT_FORMAT:
		BOLD      = '\033[1m'
		UNDERLINE = '\033[4m'
		NONE      = ''
		END         = '\033[0m'

	COLOR_CONF  = COLORS.BLUE
	FORMAT_CONF = TXT_FORMAT.BOLD
 
	print(f"{COLOR_CONF}{FORMAT_CONF}[DEV][{datetime.now()}]: {msg} {TXT_FORMAT.END}")
 
def keep_alive(node_name: str = "[place holder node name]") -> None:
	"""keeps the current node alive and continously prints the input string (default node name)

	Args:
		node_name (str, optional): the node name, can be accessed by rospy.get_name(). Defaults to "[place holder node name]".
	"""
	while (True):
		time.sleep(1)
		kill_on_ctrl_c()
		devprint(f"keeping node {node_name} alive..............................")


def kill_on_ctrl_c() -> None:
	"""
	kills the current process if a Ctrl+c signal aka. kill signal is received
	"""
	def handler(signum, frame): 
		exit(1)

	signal.signal(signal.SIGINT, handler)
