/**
* Utility Toolbox
* @author A. Lourenço, New University of Lisbon, 2011-2012.
* @see ...
* License: ...
*/


#ifndef ROS_UTILS_CONSOLE_H_
#define ROS_UTILS_CONSOLE_H_

#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/unordered_map.hpp>

using boost::assign::map_list_of;

namespace ros_utils
{


	enum COLOR { 	NORMAL,BOLD,UNDERLINED, REVERSE,
					RED_NORMAL, RED_BOLD, RED_UNDERLINED, RED_REVERSE,
					GREEN_NORMAL, GREEN_BOLD, GREEN_UNDERLINED, GREEN_REVERSE,
					YELLOW_NORMAL, YELLOW_BOLD, YELLOW_UNDERLINED, YELLOW_REVERSE,
					BLUE_NORMAL, BLUE_BOLD, BLUE_UNDERLINED, BLUE_REVERSE,
					MAGENTA_NORMAL, MAGENTA_BOLD, MAGENTA_UNDERLINED, MAGENTA_REVERSE,
					CYAN_NORMAL, CYAN_BOLD, CYAN_UNDERLINED, CYAN_REVERSE,
					WHITE_NORMAL, WHITE_BOLD, WHITE_UNDERLINED, WHITE_REVERSE	};

	const boost::unordered_map<COLOR,const char*> colorString = map_list_of
		(NORMAL, 		"\033[0m")		(BOLD, 			"\33[1m")	(UNDERLINED,	"\33[4m")			(REVERSE, 		"\33[7m")
		(RED_NORMAL, "\033[0;31m")		(RED_BOLD,  "\33[1;31m")	(RED_UNDERLINED, "\33[4;31m")		(RED_REVERSE, "\33[7;31m")
		(GREEN_NORMAL, "\033[0;32m")	(GREEN_BOLD, "\33[1;32m")	(GREEN_UNDERLINED,  "\33[4;32m")	(GREEN_REVERSE, "\33[7;32m")
		(YELLOW_NORMAL, "\033[0;33m")	(YELLOW_BOLD, "\33[1;33m")	(YELLOW_UNDERLINED, "\33[4;33m")	(YELLOW_REVERSE, "\33[7;33m")
		(BLUE_NORMAL, "\033[0;34m")		(BLUE_BOLD, "\33[1;34m")	(BLUE_UNDERLINED, "\33[4;34m")		(BLUE_REVERSE, "\33[7;34m")
		(MAGENTA_NORMAL, "\033[0;35m")	(MAGENTA_BOLD, "\33[1;35m")	(MAGENTA_UNDERLINED, "\33[4;35m")	(MAGENTA_REVERSE, "\33[7;35m")
		(CYAN_NORMAL, "\033[0;36m")		(CYAN_BOLD, "\33[1;36m")	(CYAN_UNDERLINED, "\33[4;36m")		(CYAN_REVERSE, "\33[7;36m")
		(WHITE_NORMAL, "\033[0;37m")	(WHITE_BOLD, "\33[1;37m")	(WHITE_UNDERLINED, "\33[4;37m")		(WHITE_REVERSE, "\33[7;37m");


	int ROS_PRINT (COLOR color, const char *format, ...);
    int ROS_PRINT2 (COLOR color, const char *format, ...); // with timestamp


}

#endif
