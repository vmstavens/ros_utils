
#include <string>

namespace ros_utils_cpp 
{
	namespace utils 
	{
		// print the message in a nice DEV format
		void 
		devprint(std::string msg);

		// keep the node alive
		void 
		keep_alive(std::string msg);

		// kills the node if finds signal Ctrl+c
		void 
		kill_on_ctrl_c();
	}
}
