#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "lane_detection/lane_detection.h"

using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc,argv, "lane_detection_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    LaneDetection ld(nh, pnh);
    ros::spin();
    return 0;
}
