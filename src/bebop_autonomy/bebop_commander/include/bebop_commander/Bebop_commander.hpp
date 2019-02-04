#ifndef BEBOP_WAYPOINT_HPP
#define BEBOP_WAYPOINT_HPP

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Polygon.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_datatypes.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include <ros_utils/Console.h>


namespace BC
{
    class BebopCommander
    {
    public:

        BebopCommander (const std::string group_name);
        virtual ~BebopCommander (){};

        void run();

    private:
        ros::NodeHandle                               n;
        ros::Subscriber bebop_pose_sub;
        ros::Publisher bebop_moveit_status_publisher;
        moveit::planning_interface::MoveGroupInterface     m_group;
        moveit::planning_interface::MoveGroupInterface::Plan   m_plan;

        void poseCommandClbk(const geometry_msgs::Pose::ConstPtr& msg);

    };
}

#endif
