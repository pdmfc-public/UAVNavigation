#include "bebop_planning/Bebop_planning.hpp"

using namespace BP;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bebop_planning_node");
    BP::BebopPlanning bp;
    bp.run();
    return 0;
}
