
#include <string>

namespace ros_utils_cpp 
{
	namespace CONSTANTS 
	{
		namespace COLORS
		{
			inline std::string PURPLE   = "\033[95m";
			inline std::string CYAN     = "\033[96m";
			inline std::string DARKCYAN = "\033[36m";
			inline std::string BLUE     = "\033[94m";
			inline std::string GREEN    = "\033[92m";
			inline std::string YELLOW   = "\033[93m";
			inline std::string RED      = "\033[91m";
			inline std::string NONE     = "";
		}
	
		namespace TXT_FORMAT 
		{
			inline std::string BOLD      = "\033[1m";
			inline std::string UNDERLINE = "\033[4m";
			inline std::string END       = "\033[0m";
			inline std::string NONE      = "";
			
		}
	}
}

