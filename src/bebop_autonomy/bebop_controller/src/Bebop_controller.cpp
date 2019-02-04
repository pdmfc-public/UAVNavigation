#include "bebop_controller/Bebop_controller.hpp"

using namespace BC;

BebopController::BebopController() : go(100),/*n("~")*/ action_server_(n, "multi_dof_joint_trajectory_action",
				boost::bind(&BebopController::goalCB, this, _1),
				boost::bind(&BebopController::cancelCB, this, _1),
				false),
				has_active_goal_(false){

    n.param<double>("distanceThreshold", distanceThreshold, 80);
    n.param<double>("distanceX", distanceX, 1);
    n.param<double>("distanceY", distanceY, 1);
    n.param<double>("distanceZ", distanceZ, 1);
    bebop_odom_sub = n.subscribe("/bebop/odom", 10, &BebopController::odomCallBack, this);
	bebop_cmd_vel_stop_pub = n.advertise<std_msgs::Empty>("/bebop/stop_moveit_execute", 10);
	bebop_cmd_vel_start_pub = n.advertise<std_msgs::Empty>("/bebop/start_moveit_execute", 10);
	pid_x_pub = n.advertise<std_msgs::Float64>("/desired_waypoint_x", 10);
	pid_y_pub = n.advertise<std_msgs::Float64>("/desired_waypoint_y", 10);
	pid_z_pub = n.advertise<std_msgs::Float64>("/desired_waypoint_z", 10);

	bebop_reached_desired_position_pub = n.advertise<std_msgs::Empty>("/moveit_reached_position", 10);

	created=0;
	distanceThreshold=100;
	config_callback_f = boost::bind(&BebopController::reconfigureCallback, this, _1, _2);
	config_server.setCallback(config_callback_f);

	action_server_.start();
	printf("\n\n Node ready! \n\n");
}

void BebopController::run(){

    ros::Rate go(100);


    while (ros::ok()){
        ros::spinOnce();
		publishDesiredPositions();
        go.sleep();
    }
}

void BebopController::goalCB(GoalHandle gh){
	if (has_active_goal_)
	{
		if(created){
			pthread_cancel(trajectoryExecutor);
			created=0;
		}
		bebop_cmd_vel_stop_pub.publish(empty);

		active_goal_.setCanceled();
		has_active_goal_ = false;
	}

	gh.setAccepted();
	active_goal_ = gh;
	has_active_goal_ = true;
	toExecute = gh.getGoal()->trajectory;

	if(pthread_create(&trajectoryExecutor, NULL, threadWrapper, this)==0){
		created=1;
		printf("Thread for trajectory execution created \n");
	} else {
		printf("Thread creation failed! \n");
	}

}

void BebopController::cancelCB(GoalHandle gh){
	if (active_goal_ == gh)
	{
		// Stops the controller.
		if(created){
			printf("Stop thread \n");
			pthread_cancel(trajectoryExecutor);
			created=0;
		}
		bebop_cmd_vel_stop_pub.publish(empty);

		// Marks the current goal as canceled.
		active_goal_.setCanceled();
		has_active_goal_ = false;
	}
}


void BebopController::executeTrajectory(){


    if(toExecute.joint_names[0]=="virtual_joint" && !toExecute.points.empty()){

		bebop_cmd_vel_start_pub.publish(empty);

        for(int k=0; k<toExecute.points.size() && ros::ok(); k++){

            last_wp = currentPosition;

			desired_wp = givingNextPoint(toExecute.points[k].transforms[0]);

			auto distance = ((sqrt((currentPosition.pose.pose.position.x - desired_wp.pose.pose.position.x)*(currentPosition.pose.pose.position.x - desired_wp.pose.pose.position.x) +
                            (currentPosition.pose.pose.position.y - desired_wp.pose.pose.position.y)*(currentPosition.pose.pose.position.y - desired_wp.pose.pose.position.y) +
                            (currentPosition.pose.pose.position.z - desired_wp.pose.pose.position.z)*(currentPosition.pose.pose.position.z - desired_wp.pose.pose.position.z)))
			                /
                            (sqrt((last_wp.pose.pose.position.x - desired_wp.pose.pose.position.x)*(last_wp.pose.pose.position.x - desired_wp.pose.pose.position.x) +
                          (last_wp.pose.pose.position.y - desired_wp.pose.pose.position.y)*(last_wp.pose.pose.position.y - desired_wp.pose.pose.position.y) +
                          (last_wp.pose.pose.position.z - desired_wp.pose.pose.position.z)*(last_wp.pose.pose.position.z - desired_wp.pose.pose.position.z))))*100.0;

            auto dx = abs(currentPosition.pose.pose.position.x - desired_wp.pose.pose.position.x);
            auto dy = abs(currentPosition.pose.pose.position.y - desired_wp.pose.pose.position.y);
            auto dz = abs(currentPosition.pose.pose.position.z - desired_wp.pose.pose.position.z);


            while(distance > (100-distanceThreshold) && (dx > distanceX || dy > distanceY || dz > distanceZ) && ros::ok()){

                ros::spinOnce();

				publishDesiredPositions();

                distance = ((sqrt((currentPosition.pose.pose.position.x - desired_wp.pose.pose.position.x)*(currentPosition.pose.pose.position.x - desired_wp.pose.pose.position.x) +
                                  (currentPosition.pose.pose.position.y - desired_wp.pose.pose.position.y)*(currentPosition.pose.pose.position.y - desired_wp.pose.pose.position.y) +
                                  (currentPosition.pose.pose.position.z - desired_wp.pose.pose.position.z)*(currentPosition.pose.pose.position.z - desired_wp.pose.pose.position.z)))
                            /
                            (sqrt((last_wp.pose.pose.position.x - desired_wp.pose.pose.position.x)*(last_wp.pose.pose.position.x - desired_wp.pose.pose.position.x) +
                                  (last_wp.pose.pose.position.y - desired_wp.pose.pose.position.y)*(last_wp.pose.pose.position.y - desired_wp.pose.pose.position.y) +
                                  (last_wp.pose.pose.position.z - desired_wp.pose.pose.position.z)*(last_wp.pose.pose.position.z - desired_wp.pose.pose.position.z))))*100.0;

                dx = abs(currentPosition.pose.pose.position.x - desired_wp.pose.pose.position.x);
                dy = abs(currentPosition.pose.pose.position.y - desired_wp.pose.pose.position.y);
                dz = abs(currentPosition.pose.pose.position.z - desired_wp.pose.pose.position.z);

				go.sleep();
			}

            ros_utils::ROS_PRINT(ros_utils::GREEN_BOLD,"Reached Point-> %d",k);

        }
    }
	
    std_msgs::Empty reached_msg;
    bebop_reached_desired_position_pub.publish(reached_msg);
    active_goal_.setSucceeded();
    has_active_goal_=false;
    created=0;

}

nav_msgs::Odometry BebopController::givingNextPoint(geometry_msgs::Transform_<std::allocator<void> > point) {

    nav_msgs::Odometry next_point;
	next_point.pose.pose.position.x = -point.translation.y;
	next_point.pose.pose.position.y = point.translation.x;
	next_point.pose.pose.position.z = point.translation.z;
	tf::quaternionMsgToTF(point.rotation, q);
	tf::Matrix3x3(q).getRPY(des_roll, des_pitch, des_yaw);
	next_point.pose.pose.orientation.z = des_yaw;

	return next_point;
}

void BebopController::publishDesiredPositions() {

	std_msgs::Float64 pos_x,pos_y,pos_z;

	pos_x.data = desired_wp.pose.pose.position.x;
	pos_y.data = desired_wp.pose.pose.position.y;
	pos_z.data = desired_wp.pose.pose.position.z;

	pid_x_pub.publish(pos_x);
	pid_y_pub.publish(pos_y);
	pid_z_pub.publish(pos_z);

}

void BebopController::odomCallBack(const nav_msgs::OdometryConstPtr& msg){

	currentPosition.pose.pose.position.x = -msg->pose.pose.position.y;
	currentPosition.pose.pose.position.y = msg->pose.pose.position.x;
	currentPosition.pose.pose.position.z = msg->pose.pose.position.z;
}

void BebopController::reconfigureCallback(bebop_controller::controllerConfig& config, uint32_t level){

	distanceThreshold=config.distanceThreshold;
	distanceX=config.distanceX;
	distanceY=config.distanceY;
	distanceZ=config.distanceZ;
	ros_utils::ROS_PRINT(ros_utils::BLUE_BOLD,"\nDistance Threshold-> %f \nDistanceX-> %f \nDistanceY-> %f \nDistanceZ-> %f\n",
			distanceThreshold,distanceX,distanceY,distanceZ);
}
