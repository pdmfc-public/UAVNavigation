#ifndef BEBOP_CONTROLLER_HPP
#define BEBOP_CONTROLLER_HPP

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pthread.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <bebop_controller/MultiDofFollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <ros_utils/Console.h>
#include <dynamic_reconfigure/server.h>
#include <bebop_controller/controllerConfig.h>

namespace BC
{
    class BebopController
    {
    public:

        BebopController ();
        virtual ~BebopController (){};

        void run();

    private:
        ros::NodeHandle                               n;
        ros::Subscriber bebop_odom_sub;
        ros::Publisher bebop_cmd_vel_start_pub,bebop_cmd_vel_stop_pub,pid_x_pub, pid_y_pub, pid_z_pub,bebop_reached_desired_position_pub;
        typedef actionlib::ActionServer<bebop_controller::MultiDofFollowJointTrajectoryAction> ActionServer;
        typedef ActionServer::GoalHandle GoalHandle;
        ActionServer action_server_;

        nav_msgs::Odometry desired_wp, last_wp;
        nav_msgs::Odometry currentPosition;

        tf::Quaternion q;
        double des_roll, des_pitch, des_yaw;

        std_msgs::Empty empty;


        pthread_t trajectoryExecutor;
        int created;

        bool has_active_goal_;
        GoalHandle active_goal_;
        trajectory_msgs::MultiDOFJointTrajectory_<std::allocator<void> > toExecute;

        tf::TransformListener listener;

        ros::Rate go;
        double distanceThreshold,distanceX,distanceY,distanceZ;
        dynamic_reconfigure::Server<bebop_controller::controllerConfig> config_server;
        dynamic_reconfigure::Server<bebop_controller::controllerConfig>::CallbackType config_callback_f;

        static void* threadWrapper(void* arg) {
            BebopController * mySelf=(BebopController*)arg;
            mySelf->executeTrajectory();
            return NULL;
        }

        void goalCB(GoalHandle gh);
        void cancelCB(GoalHandle gh);
        void executeTrajectory();
        nav_msgs::Odometry givingNextPoint(geometry_msgs::Transform_<std::allocator<void> > point);
        void publishDesiredPositions();
        void odomCallBack(const nav_msgs::OdometryConstPtr& msg);
        void reconfigureCallback(bebop_controller::controllerConfig& config, uint32_t level);
    };
}

#endif
