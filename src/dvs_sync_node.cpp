#include "dvs_sync.h"

#include <ros/ros.h>

using namespace dvs_sync;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "dvs_sync_node");
    ros::NodeHandle nh("~");

    auto config_file = nh.param<std::string>("config_file", "");

    DvsSync dvs_sync(config_file);
    dvs_sync.run();

    return 0;
}