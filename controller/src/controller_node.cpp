#include <ros/ros.h>

#include "controller/controller.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    Controller ctrl(nh, pnh);
    ros::spin();
    return 0;
}
