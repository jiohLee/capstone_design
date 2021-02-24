#include <ros/ros.h>

#include "obstacle_detection/obstacle_detection.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_detection_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ObstacleDetection obs(nh, pnh);

    ros::spin();
    return 0;
}
