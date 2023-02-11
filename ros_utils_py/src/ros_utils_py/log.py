#!/usr/bin/env python3

from ros_utils_py import devprint, COLORS, LOG_LEVELS
from datetime import date
import rospkg
import os

class Logger():

	def __init__(self, file_name: str = ""):
		self._log_dir = self._init_log_dir(file_name=file_name)

	def info(self, msg: str) -> None:
		if self._log_dir != "":
			devprint(msg + "\n", file_path=self._log_dir,color=COLORS.BLUE, log_level=LOG_LEVELS.INFO)
		devprint(msg, color=COLORS.BLUE, log_level=LOG_LEVELS.INFO)

	def warn(self, msg: str) -> None:
		if self._log_dir != "":
			devprint(msg + "\n", file_path=self._log_dir,color=COLORS.YELLOW, log_level=LOG_LEVELS.WARN)
		devprint(msg, color=COLORS.YELLOW, log_level=LOG_LEVELS.WARN)

	def error(self, msg: str) -> None:
		if self._log_dir != "":
			devprint(msg + "\n", file_path=self._log_dir,color=COLORS.RED, log_level=LOG_LEVELS.ERROR)
		devprint(msg, color=COLORS.RED, log_level=LOG_LEVELS.ERROR)

	def success(self, msg: str) -> None:
		if self._log_dir != "":
			devprint(msg + "\n", file_path=self._log_dir,color=COLORS.GREEN, log_level=LOG_LEVELS.SUCCESS)
		devprint(msg, color=COLORS.GREEN, log_level=LOG_LEVELS.SUCCESS)

	def _init_log_dir(self, file_name: str) -> str:
		"""This creates a log folder for logging files in the package folder with the node calling this method. The input should simply be __file__. This function should only be called once in the beginning of the node.
		Args:
			file_name (str): the name of the current file i.e. __file__
		Return:
			log file path
		"""
		if file_name == "":
			return ""
	
		# get the package name which contains the file file_name
		pkg_name = rospkg.get_package_name( file_name )

		# get the corresponding package path
		pkg_path:str = rospkg.RosPack().get_path(pkg_name) + "/"

		devprint("I have found this package: " + pkg_path, color=COLORS.GREEN)

		logging_dir:str = pkg_path + "log/"

		# create pkg/log/
		if not os.path.exists(logging_dir):
			devprint("The logging dir did not exist and i therefore created: " + logging_dir, color=COLORS.GREEN)
			os.mkdir(logging_dir)

		# create pkg/log/<date>.log
  
		devprint("logging_dir = " + logging_dir)
  
		date_str = date.today().strftime("%Y%m%d_%H%M%S")
		pkg_name_str = logging_dir.split("/")[-3]
		file_name_str = file_name.split("/")[-1]
		file_name_str_no_f_type = '.'.join(file_name_str.split(".")[:-1])
		log_file_name = logging_dir + date_str + "_" + pkg_name_str + "_" + file_name_str_no_f_type + ".log"

		# if not os.path.exists(file_name):
		open(log_file_name,"w").close()

		# clear file content


		# hey this is the logging dir for the file
		return log_file_name