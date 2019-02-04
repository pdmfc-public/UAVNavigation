#ifndef BEBOP_WAYPOINT_HPP
#define BEBOP_WAYPOINT_HPP

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros_utils/Console.h>
#include <std_msgs/Bool.h>

namespace BWP
{
    class BebopWayPoint
    {
    public:

        BebopWayPoint ();
        virtual ~BebopWayPoint (){};

        void run();

    private:
        ros::NodeHandle                               n;
        ros::Subscriber bebop_odom_sub, pid_control_effor_x_sub,pid_control_effor_y_sub,pid_control_effor_z_sub,bebop_stop_execute_sub,bebop_start_execute_sub;
        ros::Publisher pid_enable_pub,pid_current_position_x_pub,pid_current_position_y_pub,pid_current_position_z_pub,cmd_vel_pub;

        geometry_msgs::Twist msg_cmd_vel;

        tf::TransformListener listener;
        tf::StampedTransform transform;

        void controlEffortX(const std_msgs::Float64ConstPtr& msg);
        void controlEffortY(const std_msgs::Float64ConstPtr& msg);
        void controlEffortZ(const std_msgs::Float64ConstPtr& msg);
        void startBebopExecute(const std_msgs::EmptyConstPtr& msg);
        void stopBebopExecute(const std_msgs::EmptyConstPtr& msg);
	void publishCurrentPosition(const nav_msgs::OdometryConstPtr& msg);
    };
}

#endif
