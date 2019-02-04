#include "bebop_waypoint/Bebop_waypoint.hpp"

using namespace BWP;

BebopWayPoint::BebopWayPoint():n("~") {

    bebop_start_execute_sub = n.subscribe("/bebop/start_moveit_execute", 10, &BebopWayPoint::startBebopExecute, this);
    bebop_stop_execute_sub = n.subscribe("/bebop/stop_moveit_execute", 10, &BebopWayPoint::stopBebopExecute, this);
    bebop_odom_sub = n.subscribe("/bebop/odom", 10, &BebopWayPoint::publishCurrentPosition, this);
    

    pid_current_position_x_pub = n.advertise<std_msgs::Float64>("/state_x", 10);
    pid_current_position_y_pub = n.advertise<std_msgs::Float64>("/state_y", 10);
    pid_current_position_z_pub = n.advertise<std_msgs::Float64>("/state_z", 10);
    pid_enable_pub = n.advertise<std_msgs::Bool>("/pid_enable", 10);

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 10);

}


void BebopWayPoint::run(){

    ros::Rate go(100);
    while (ros::ok()){
	ros::spinOnce();
        cmd_vel_pub.publish(msg_cmd_vel);
        go.sleep();
    }
}

void BebopWayPoint::publishCurrentPosition(const nav_msgs::OdometryConstPtr& msg){

	std_msgs::Float64 pos_x, pos_y, pos_z;
        pos_x.data = -msg->pose.pose.position.y;
        pos_y.data = msg->pose.pose.position.x;
        pos_z.data = msg->pose.pose.position.z;

        pid_current_position_x_pub.publish(pos_x);
        pid_current_position_y_pub.publish(pos_y);
        pid_current_position_z_pub.publish(pos_z);
}

void BebopWayPoint::controlEffortX(const std_msgs::Float64ConstPtr& msg){

    msg_cmd_vel.linear.x = msg->data;

}

void BebopWayPoint::controlEffortY(const std_msgs::Float64ConstPtr& msg){

    msg_cmd_vel.linear.y = msg->data;

}

void BebopWayPoint::controlEffortZ(const std_msgs::Float64ConstPtr& msg){

    msg_cmd_vel.linear.z = msg->data;

}

void BebopWayPoint::startBebopExecute(const std_msgs::EmptyConstPtr& msg){
    
    pid_control_effor_x_sub = n.subscribe("/control_effort_x", 10, &BebopWayPoint::controlEffortX, this);
    pid_control_effor_y_sub = n.subscribe("/control_effort_y", 10, &BebopWayPoint::controlEffortY, this);
    pid_control_effor_z_sub = n.subscribe("/control_effort_z", 10, &BebopWayPoint::controlEffortZ, this);

    std_msgs::Bool pid_msg; pid_msg.data = true;
    pid_enable_pub.publish(pid_msg);
    ros_utils::ROS_PRINT(ros_utils::MAGENTA_BOLD,"Start Execute!");
}

void BebopWayPoint::stopBebopExecute(const std_msgs::EmptyConstPtr& msg){

    msg_cmd_vel.linear.x=0;
    msg_cmd_vel.linear.y=0;
    msg_cmd_vel.linear.z=0;

    pid_control_effor_x_sub.shutdown();
    pid_control_effor_y_sub.shutdown();
    pid_control_effor_z_sub.shutdown();

    std_msgs::Bool pid_msg; pid_msg.data = false;
    pid_enable_pub.publish(pid_msg);
    ros_utils::ROS_PRINT(ros_utils::RED_BOLD,"Stop Execute!");
}
