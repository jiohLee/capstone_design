#ifndef LANE_DETECTION_H
#define LANE_DETECTION_H

#include <vector>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Point.h>
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
    ros::Publisher pubLaneCenterPoint;
    ros::Publisher pubOnLane;

    // ROS messages
    geometry_msgs::Point laneCenterPoint;
    std_msgs::String onLane;

    // ROS Param
    int leftHigh;
    int leftLow;

    int whiteHueMax;
    int whiteHueMin;
    int whiteSatMax;
    int whiteSatMin;
    int whiteValMax;
    int whiteValMin;

    int yellowHueMax;
    int yellowHueMin;
    int yellowSatMax;
    int yellowSatMin;
    int yellowValMax;
    int yellowValMin;

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
    Mat topViewBinWhite;
    Mat topViewBinYellow;

    std::vector<Point2f> srcTri;
    std::vector<Point2f> dstTri;

    std::vector<Point> centeroidsYellow;
    std::vector<Point> centeroidsWhite;

    ros::Time timePointPrev;
    double timePointElapsed;

    // constant
    const int row__ = 480;
    const int col__ = 864;
    const float px_per_m = 480;
};

#endif // LANE_DETECTION_H
