#include "bebop_planning/Bebop_planning.hpp"

using namespace BP;

BebopPlanning::BebopPlanning():n("~") {

    n.param<std::string>("ip", ip, "192.168.42.1");
    n.param<std::string>("user", user, "anonymous");
    n.param<std::string>("pass", pass, "");
    n.param<int>("port", port, 21);

    donwloadMavLinkFile_sub= n.subscribe("/downloadMavlinkFile", 10, &BebopPlanning::downloadMavLinkFile, this);
    uploadMavLinkFile_sub= n.subscribe("/uploadMavlinkFile", 10, &BebopPlanning::uploadMavLinkFile, this);
    bebop_plan_sub= n.subscribe("/doTheplan", 10, &BebopPlanning::doThePlan, this);
    ftp_connect_server_sub = n.subscribe("/connectFTPServer", 10, &BebopPlanning::connectFTPServer, this);
    reached_position_sub = n.subscribe("/moveit_reached_position", 10, &BebopPlanning::reachedPosition, this);
    bebop_gps_sub = n.subscribe("/bebop/states/ardrone3/PilotingState/PositionChanged", 10,
            &BebopPlanning::gpsCallBack, this);
    pose_command_moveit_pub = n.advertise<geometry_msgs::Pose>("/pose_command", 1000);

    // Connect to the server
    ftp.connect(ip, (ushort)port,sf::seconds(5));
    ros_utils::ROS_PRINT(ros_utils::GREEN_BOLD, "FTP Server connected. Now trying to login...");
    //Once you're connected to the server, the next step is to authenticate yourself:
    if (ftp.login(user, pass).isOk()) {
        ros_utils::ROS_PRINT(ros_utils::GREEN_BOLD, "FTP Server login Success");
        ftp.keepAlive();
        already_connected = true;
    }
    else
        ros_utils::ROS_PRINT(ros_utils::RED_BOLD,"FTP Server login failed");

}

void BebopPlanning::run(){

    ros::Rate go(100);
    while (ros::ok()){
        ros::spinOnce();
        go.sleep();
    }
    ftp.disconnect();
}

void BebopPlanning::downloadMavLinkFile(const std_msgs::StringConstPtr& msg){

    std::stringstream planning_to_path, planning_from_path;
    planning_to_path << ros::package::getPath("bebop_planning") << "/Plans/";
    planning_from_path << "/internal_000/flightplans/" << msg->data << ".mavlink";

    if(ftp.download(planning_from_path.str(), planning_to_path.str(), sf::Ftp::Ascii).isOk())
        ros_utils::ROS_PRINT(ros_utils::GREEN_BOLD,"Donwload Success");
    else
        ros_utils::ROS_PRINT(ros_utils::RED_BOLD,"Fail to Download");

}

void BebopPlanning::uploadMavLinkFile(const std_msgs::StringConstPtr& msg){

    std::stringstream planning_path;
    planning_path << ros::package::getPath("bebop_planning") << "/Plans/"<< msg->data <<".mavlink";

    if(ftp.upload(planning_path.str(),"/internal_000/flightplans/",sf::Ftp::Ascii).isOk())
        ros_utils::ROS_PRINT(ros_utils::GREEN_BOLD,"Upload Success");
    else
        ros_utils::ROS_PRINT(ros_utils::RED_BOLD,"Fail to Upload");

}

void BebopPlanning::connectFTPServer(const bebop_planning::connectFTPConstPtr& msg){

    if(already_connected) {
        ftp.disconnect();
        already_connected = false;
    }

    ip = msg->ip;
    user = msg->user;
    pass = msg->pass;
    port = msg->port;

    ftp.connect(ip, (ushort)port, sf::seconds(5));

    ros_utils::ROS_PRINT(ros_utils::GREEN_BOLD, "FTP Server connected. Now trying to login...");
    //Once you're connected to the server, the next step is to authenticate yourself:
    if (ftp.login(user, pass).isOk()) {
        ros_utils::ROS_PRINT(ros_utils::GREEN_BOLD, "FTP Server login Success");
        ftp.keepAlive();
        already_connected = true;
    }
    else
        ros_utils::ROS_PRINT(ros_utils::RED_BOLD,"FTP Server login failed");

}

void BebopPlanning::doThePlan(const std_msgs::StringConstPtr& msg){

    current_desired_position=0;
    plan_vector.clear();

    std::fstream fs;
    std::stringstream file_path;

    file_path << ros::package::getPath("bebop_planning") << "/Plans/"<<msg->data<<".mavlink";
    fs.open (file_path.str(), std::fstream::in | std::fstream::out | std::fstream::app);

    //fs.seekg(0 ,std::ios_base::beg);
    std::string line;
    while (getline(fs ,line)) {
        //std::cout << line << std::endl;
        std::vector<std::string> results;
        boost::split(results, line, [](char c) { return c == '\t'; });
        if(results.size() < 4)
            continue;
        if(results[3]!="16")
            continue;
        plan_vector.push_back(line);
    }

    number_of_lines = plan_vector.size();
    //std::cout << number_of_lines << std::endl;
    fs.close();

    std::vector<std::string> results;
    boost::split(results, plan_vector[current_desired_position], [](char c){return c == '\t';});
    last_position_x = current_position_x;
    last_position_y = current_position_y;
    publishDesiredPosition(results);

}

void BebopPlanning::reachedPosition(const std_msgs::EmptyConstPtr& msg){

    current_desired_position++;

    if(current_desired_position < number_of_lines) {
        std::vector<std::string> results;
        boost::split(results, plan_vector[current_desired_position], [](char c) { return c == '\t'; });
        last_position_x = current_position_x;
        last_position_y = current_position_y;
        publishDesiredPosition(results);
    }
    else
        ros_utils::ROS_PRINT(ros_utils::GREEN_BOLD, "End plan with Success");

}

void BebopPlanning::publishDesiredPosition(std::vector<std::string> plan){

    std::string UTMZone;

    gps_common::LLtoUTM(std::stod(plan[8]), std::stod(plan[9]), current_position_x, current_position_y,UTMZone);

    geometry_msgs::Pose msg;
    msg.position.x = (current_position_x-last_position_x);
    msg.position.y = -(current_position_y-last_position_y);
    msg.position.z = std::stod(plan[10]);
    msg.orientation.z = std::stod(plan[7]);
    msg.orientation.w = 1;

    pose_command_moveit_pub.publish(msg);

}

void BebopPlanning::gpsCallBack(const bebop_msgs::Ardrone3PilotingStatePositionChangedConstPtr& msg){

    std::string UTMZone;

    gps_common::LLtoUTM(msg->latitude, msg->longitude, current_position_x, current_position_y,UTMZone);

}




