#ifndef LANE_DETECTION_H
#define LANE_DETECTION_H

#include <vector>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Point.h>

using namespace cv;

class LaneDetection
{
public:
    LaneDetection(ros::NodeHandle & nh, ros::NodeHandle & pnh);
    void saveLastImage();
private:

    // ROS Service
    ros::Subscriber subCompImg;
    ros::Publisher pubPoint;

    geometry_msgs::Point pt;

    void imgCallback(const sensor_msgs::CompressedImage::ConstPtr & msg);


    // ROS Param
    bool save_last_image;
    int hue_1_max;
    int hue_1_min;
    int hue_2_max;
    int hue_2_min;
    int sat_max;
    int sat_min;
    int val_max;
    int val_min;
    bool show_source;
    bool show_reduced;
    bool show_sliding_window;
    int waypoint_height;

    // Variables
    Mat src;
    Mat birdEye;
    Mat binaryImg;


    std::vector<Point2f> srcTri;
    std::vector<Point2f> dstTri;
};

#endif // LANE_DETECTION_H
