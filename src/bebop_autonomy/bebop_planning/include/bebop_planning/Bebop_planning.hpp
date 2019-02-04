#ifndef BEBOP_PLANNING_HPP
#define BEBOP_PLANNING_HPP

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros_utils/Console.h>
#include <std_msgs/Bool.h>
#include <curl/curl.h>
#include <SFML/Network.hpp>
#include <SFML/System/Time.hpp>
#include "bebop_planning/connectFTP.h"
#include <fstream>
#include <gps_common/conversions.h>
#include <bebop_msgs/Ardrone3PilotingStatePositionChanged.h>

namespace BP
{
    class BebopPlanning
    {
    public:

        BebopPlanning ();
        virtual ~BebopPlanning (){};

        void run();

    private:
        ros::NodeHandle n;
        std::string ip,user,pass;
        bool already_connected = false;
        int port;
        ros::Subscriber bebop_gps_sub,donwloadMavLinkFile_sub,uploadMavLinkFile_sub,
                bebop_plan_sub,ftp_connect_server_sub,reached_position_sub;
        ros::Publisher pose_command_moveit_pub;

        // Create a new FTP client
        sf::Ftp ftp;

        //Plan
        long number_of_lines=0,current_desired_position;
        std::vector<std::string> plan_vector;
        nav_msgs::Odometry currentPosition;
        double last_position_x,last_position_y,current_position_y,current_position_x;
        bool started_plan=false;

        void downloadMavLinkFile(const std_msgs::StringConstPtr& msg);
        void uploadMavLinkFile(const std_msgs::StringConstPtr& msg);
        void doThePlan(const std_msgs::StringConstPtr& msg);
        void connectFTPServer(const bebop_planning::connectFTPConstPtr& msg);
        void reachedPosition(const std_msgs::EmptyConstPtr& msg);
        void publishDesiredPosition(std::vector<std::string> plan);
        void gpsCallBack(const bebop_msgs::Ardrone3PilotingStatePositionChangedConstPtr& msg);

    };
}

#endif
