#!/usr/bin/env python3

from datetime import datetime
import time

def devprint(msg: str) -> None:
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
	while (True):
		time.sleep(1)
		devprint(f"keeping node {node_name} alive..............................")
