#include <cstdio>
#include <thread>
#include "ros_utils_cpp/utils.hpp"
#include "ros_utils_cpp/constants.hpp"
#include <iostream>
#include <cstdlib>
#include <signal.h>

namespace ros_utils_cpp 
{
	namespace utils 
	{
		void 
		devprint(std::string msg)
		{
			std::string COLOR_CONF = CONSTANTS::COLORS::BLUE;
			std::string FORMAT_CONF = CONSTANTS::TXT_FORMAT::BOLD;
			time_t curtime;

			// get time and date
			std::string now = std::string(ctime(&curtime));

			// print the msg with format
			std::cout << COLOR_CONF << FORMAT_CONF << "[DEV] " << now << " " << msg << std::endl;
		}

		void 
		keep_alive(std::string msg)
		{
			while (true) 
			{
				// kill node if finds signal Ctrl+c
				kill_on_ctrl_c();

				size_t num_of_milli_seconds = 1000; // 1 s.

				// sleep 1 s.
				std::this_thread::sleep_for(std::chrono::milliseconds(num_of_milli_seconds));

				// devprint
				devprint("keeping node " + msg + " alive..............................");
			}
		}

		void
		kill_on_ctrl_c()
		{
			struct sigaction sigIntHandler;

			sigIntHandler.sa_handler = [] (int signal) { printf("Caught signal %d\n",signal); exit(1);  };

			sigemptyset(&sigIntHandler.sa_mask);

			sigIntHandler.sa_flags = 0;

			sigaction(SIGINT, &sigIntHandler, NULL);
		}
	}
}
