#include "bebop_waypoint/Bebop_waypoint.hpp"

using namespace BWP;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bebop_waypoint_node");
    BWP::BebopWayPoint bw;
    bw.run();
    return 0;
}
