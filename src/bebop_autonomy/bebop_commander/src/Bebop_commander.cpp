#include "bebop_commander/Bebop_commander.hpp"

using namespace BC;

BebopCommander::BebopCommander(const std::string group_name):n("~"), m_group(group_name) {

    bebop_pose_sub = n.subscribe("/pose_command", 1, &BebopCommander::poseCommandClbk, this );
    bebop_moveit_status_publisher = n.advertise<moveit_msgs::MoveItErrorCodes>("/moveit_status", 1000);

    m_group.setPlannerId("RRTConnectkConfigDefault");
    m_group.setPlanningTime(5.0);
    m_group.setWorkspace  ( -300,-300,-300,300,300,300);

    ros_utils::ROS_PRINT(ros_utils::WHITE_NORMAL, "Planning frame: %s", m_group.getPlanningFrame().c_str());
    ros_utils::ROS_PRINT(ros_utils::WHITE_NORMAL, "End Effector frame: %s", m_group.getEndEffectorLink().c_str());

    m_group.setStartStateToCurrentState();
    m_group.allowReplanning(true);
    m_group.setGoalPositionTolerance(0.0001);
    m_group.setGoalOrientationTolerance(0.5);
    m_group.setPlanningTime(5.0);
    m_group.setNumPlanningAttempts(10);
    m_group.setMaxVelocityScalingFactor(1);
    m_group.setMaxAccelerationScalingFactor(1);
    m_group.setGoalTolerance(0.5);
    m_group.setGoalJointTolerance(0.0001);

}


void BebopCommander::run(){

    ros_utils::ROS_PRINT(ros_utils::WHITE_BOLD, "MoveIt! Commander Ready");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
}

void BebopCommander::poseCommandClbk(const geometry_msgs::Pose::ConstPtr& msg){

    auto current_join_values = m_group.getCurrentJointValues();

    m_group.setWorkspace(current_join_values[0]-300,current_join_values[1]-300,0,
                         current_join_values[0]+300,current_join_values[1]+300,current_join_values[2]+300);

    m_group.clearPathConstraints();
    ros_utils::ROS_PRINT(ros_utils::CYAN_NORMAL, "Received Pose Command (%s): (%.2f , %.2f , %.2f) (%.5f , %.5f , %.5f , %.5f)", m_group.getEndEffectorLink().c_str(), msg->position.x, msg->position.y, msg->position.z, msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    tf::Quaternion q;
    quaternionMsgToTF(msg->orientation, q);
    q.normalize();

    std::vector<double> joint_group_positions;
    joint_group_positions.push_back(msg->position.x);
    joint_group_positions.push_back(msg->position.y);
    joint_group_positions.push_back(msg->position.z);
    joint_group_positions.push_back(q.getX());
    joint_group_positions.push_back(q.getY());
    joint_group_positions.push_back(q.getZ());
    joint_group_positions.push_back(q.getW());

    m_group.setJointValueTarget(joint_group_positions);

    moveit_msgs::MoveItErrorCodes error_msg;
    moveit::planning_interface::MoveItErrorCode plan_error_code = m_group.plan(m_plan);

    if(plan_error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ros_utils::ROS_PRINT(ros_utils::GREEN_NORMAL, "Move Group found a Plan (now executing...)");
        moveit::planning_interface::MoveItErrorCode execute_error_code = m_group.execute(m_plan);
        error_msg.val = execute_error_code.val;
        bebop_moveit_status_publisher.publish(error_msg);

        if(execute_error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            ros_utils::ROS_PRINT(ros_utils::GREEN_NORMAL, "Move Group executed the Plan Successfully");
        else
            ros_utils::ROS_PRINT(ros_utils::RED_NORMAL, "Move Group failed to execute the Plan (ERROR_CODE %d)", (int) plan_error_code.val);
    }
    else
    {
        error_msg.val = plan_error_code.val;
        bebop_moveit_status_publisher.publish(error_msg);
        ros_utils::ROS_PRINT(ros_utils::RED_NORMAL, "Move Group failed to find a Plan (ERROR_CODE %d)" , (int) plan_error_code.val);
    }

}

