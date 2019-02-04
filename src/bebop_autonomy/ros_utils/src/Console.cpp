#include <ros_utils/Console.h>

namespace ros_utils
{
	int ROS_PRINT(COLOR color, const char *format, ...)
	{
		va_list arg;
		int done;
		std::string this_node = ros::this_node::getName();
		boost::to_upper(this_node);
		fprintf (stdout, "%s[%s] [%.3f]:%s ", colorString.at(color), this_node.c_str(), ros::WallTime::now().toSec(), colorString.at(color));
		va_start (arg, format);
		done = vfprintf (stdout, format, arg);
		fprintf (stdout,"%s\n", colorString.at(NORMAL));
		va_end (arg);

		return done;
	}
}
