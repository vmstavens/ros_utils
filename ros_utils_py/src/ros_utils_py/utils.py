#!/usr/bin/env python3

from datetime import datetime

def devprint(msg: str) -> None:
	class color_conf:
		PURPLE = '\033[95m'
		CYAN = '\033[96m'
		DARKCYAN = '\033[36m'
		BLUE = '\033[94m'
		GREEN = '\033[92m'
		YELLOW = '\033[93m'
		RED = '\033[91m'
		NONE = ''

	class format_conf:
		BOLD = '\033[1m'
		UNDERLINE = '\033[4m'

	END = '\033[0m'
	NONE = ''

	COLOR_CONF = color_conf.BLUE
	FORMAT_CONF = format_conf.BOLD
	print(f"{COLOR_CONF}{FORMAT_CONF}[DEV][{datetime.now()}]: {msg} {END}")