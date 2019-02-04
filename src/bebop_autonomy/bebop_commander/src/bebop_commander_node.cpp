#include "bebop_commander/Bebop_commander.hpp"

using namespace BC;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bebop_commander_node");
    std::string group_name;
    ros::param::param<std::string>( ros::this_node::getName()+"/group_name", group_name, "Bebop_base" );
    BC::BebopCommander bmc(group_name);
    bmc.run();
    return 0;
}
