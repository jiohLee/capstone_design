#ifndef LANE_DETECTION_H
#define LANE_DETECTION_H

#include <vector>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

using namespace cv;

class LaneDetection
{
public:

    LaneDetection(ros::NodeHandle & nh, ros::NodeHandle & pnh);

private:

    // ROS Callbacks
    void imgCallback(const sensor_msgs::CompressedImage::ConstPtr & msg);

    // ROS Service
    ros::NodeHandle & nh;
    ros::NodeHandle & pnh;

    ros::Subscriber subCompImg;
    ros::Publisher pubTargetSteer;
    ros::Publisher pubOnLane;

    // ROS messages
    std_msgs::Float64 targetSteer;
    std_msgs::String onLane;

    // ROS Param
    int leftHigh;
    int leftLow;

    int redHMax1;
    int redHMin1;
    int redHMax2;
    int redHMin2;
    int yellowHMax;
    int yellowHMin;

    int satMax;
    int satMin;
    int valMax;
    int valMin;

    int windowWidth;
    int windowNum;
    int targetWindowHeight;

    bool showSource;
    bool showSlidingWindow;

    // Functions
    void getSlidingWindow(Mat& input, std::vector<Point>& centeroids, int windowWidth, int windowNum);
    void drawSlidingWindow(Mat& input, std::vector<Point>& centeroids, int windowWidth);
    void erodeAndDilate(Mat& input, int shape, Size kSize, int repeat);

    // Connected Component Index
    enum StatsIdx
    {
        LEFT_TOP_X = 0,
        LEFT_TOP_Y,
        WIDTH,
        HEIGHT,
        PIXELS
    };

    enum CenteroidIdx
    {
        CENTER_X = 0,
        CENTER_Y
    };

    // Variables
    Mat src;
    Mat topView;
    Mat topViewBinRed;
    Mat topViewBinYellow;

    std::vector<Point2f> srcTri;
    std::vector<Point2f> dstTri;

    std::vector<Point> centeroidsYellow;
    std::vector<Point> centeroidsRed;

    ros::Time timePointPrev;
    double timePointElapsed;
};

#endif // LANE_DETECTION_H
