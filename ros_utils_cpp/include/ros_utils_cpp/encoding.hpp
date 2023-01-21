
#include <string>

namespace ros_utils_cpp 
{
	namespace encoding 
	{
		// base 64 encoder
		std::string 
		base64_encode(const std::string &in);

		// base 64 decoder
		std::string 
		base64_decode(const std::string &in);
	}
}

