#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include "capstone_utils/capstone_utils.h"

using namespace cv;

typedef struct hsvRange
{
    int h_u;
    int h_l;
    int s_u;
    int s_l;
    int v_u;
    int v_l;
}hsvRange;

void onTrack(int, void*){}

class HSVBinarization
{
public:
    HSVBinarization(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    // Functions
    void printParams();
private:

    // ROS Callbacks
    void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);

    // ROS Service
    ros::NodeHandle & nh;
    ros::NodeHandle & pnh;

    ros::Subscriber subCompImg;

    // Variables
    std::string winName;
    hsvRange r;
    Mat src;
};

HSVBinarization::HSVBinarization(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh(nh)
    , pnh(pnh)
    , r({0,0,0,0,0,0})
    , winName("hsv binarization")
{
    std::string source_name;
    pnh.param<std::string>("source_name", source_name, "usb_cam_1/image_raw/compressed");

    subCompImg = nh.subscribe(source_name, 1, &HSVBinarization::imageCallback, this);

    namedWindow(winName, WINDOW_AUTOSIZE);

    r = {0,0,0,0,0,0};
    createTrackbar("h_u", winName, &r.h_u, 180, onTrack);
    createTrackbar("h_l", winName, &r.h_l, 180, onTrack);
    createTrackbar("s_u", winName, &r.s_u, 255, onTrack);
    createTrackbar("s_l", winName, &r.s_l, 255, onTrack);
    createTrackbar("v_u", winName, &r.v_u, 255, onTrack);
    createTrackbar("v_l", winName, &r.v_l, 255, onTrack);
}

void HSVBinarization::imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    src = cvPtr->image;

    Mat result;
    Mat hsv;
    cvtColor(src, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(r.h_l, r.s_l, r.v_l), Scalar(r.h_u, r.s_u, r.v_u), result);

    cvtColor(result, result, COLOR_GRAY2BGR);
    Mat merged = mergeImage(1, 2, {src, result});

    imshow(winName, merged);
    waitKey(1);
}

void HSVBinarization::printParams()
{
    std::cout << "H range : " << r.h_l << " to " << r.h_u << "\n"
              << "S range : " << r.s_l << " to " << r.s_u << "\n"
              << "V range : " << r.v_l << " to " << r.v_u << "\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hsv_bizarization_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    HSVBinarization hsvb(nh, pnh);
    ros::spin();
    hsvb.printParams();
    return 0;
}
