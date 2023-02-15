#!/usr/bin/env python3

from datetime import datetime
import time
import signal
import rospkg
from ros_utils_py.log import Logger
import os

__utils_logger = Logger()

class COLORS:
	"""should only be used as input for devprint() """
	PURPLE    = '\033[95m'
	CYAN      = '\033[96m'
	DARKCYAN  = '\033[36m'
	BLUE      = '\033[94m'
	GREEN     = '\033[92m'
	YELLOW    = '\033[93m'
	RED       = '\033[91m'
	NONE      = ''

class TXT_FORMAT:
	"""should only be used as input for devprint() """
	BOLD      = '\033[1m'
	UNDERLINE = '\033[4m'
	NONE      = ''
	END         = '\033[0m'

class LOG_LEVELS:
	INFO  = "INFO"
	WARN  = "WARN"
	ERROR = "ERROR"
	SUCCESS  = "SUCCESS"

def devprint(msg: str, file_path:str = "", color: str = COLORS.BLUE, log_level: str = LOG_LEVELS.INFO, format: str = TXT_FORMAT.BOLD) -> None:
	"""prints the input string in an easy to spot format when scrolling through the ros gazebo cluttered terminal. The default formatting is blue and bold text, but can be configured as pleased.

	Args:
		msg (str): the string to be printed
	"""

	COLOR_CONF  = color
	FORMAT_CONF = format
	
	# if a file_path is given, write the message to the file
	if file_path != "":
		f = open(file_path, "a")
		f.write(f"[DEV, {log_level}][{datetime.now()}]: {msg}")
		f.close()
		return
	
	print(f"{COLOR_CONF}{FORMAT_CONF}[DEV, {log_level}][{datetime.now()}]: {msg} {TXT_FORMAT.END}")
 
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

def create_pkg_dir(current_file_name: str, name_of_folder: str = "log/") -> str:
	"""creates a folder in the package directory

	Args:
		current_file_name (str): the current file's name i.e. __file__
		name_of_folder (str): the name of the folder you want created

	Returns:
		bool: did the creation complete successfully
	"""

	# make sure the folder name has the right "/" ending
	name_of_folder = name_of_folder + "/" if name_of_folder[-1] != "/" else name_of_folder

	# get the package name which contains the file file_name
	pkg_name = rospkg.get_package_name(current_file_name)

	# get the corresponding package path
	pkg_path: str = rospkg.RosPack().get_path(pkg_name) + "/"

	__utils_logger.success("I have found this package: " + pkg_path)

	data_dir: str = pkg_path + "data/"

	# create pkg/log/
	if not os.path.exists(data_dir):
		__utils_logger.info("The logging dir did not exist and i therefore created: " + data_dir)
		os.mkdir(data_dir)
  
	return data_dir
