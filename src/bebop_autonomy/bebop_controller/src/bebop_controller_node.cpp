#include "bebop_controller/Bebop_controller.hpp"

using namespace BC;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bebop_controller_node");
    BC::BebopController bcn;
    bcn.run();
    return 0;
}
