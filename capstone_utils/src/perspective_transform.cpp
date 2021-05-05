#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include "capstone_utils/capstone_utils.h"

using namespace cv;

class PerspectiveTransform
{
public:
    PerspectiveTransform(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    // Functions
    void printParam();

    // Variables
    std::string winName;
    std::vector<Point2f> points;
    Mat frame;
    Mat result;

private:
    // ROS Callbacks
    void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);

    // ROS Service
    ros::NodeHandle & nh;
    ros::NodeHandle & pnh;

    ros::Subscriber subCompImg;

    // Variables
    std::vector<Point2f> src;
    std::vector<Point2f> dst;
    Mat warp;
};

void onMouse(int event, int x, int y, int flag, void* userdata)
{
    PerspectiveTransform & pt = *static_cast<PerspectiveTransform*>(userdata);

    Point msp(x, y);
    Rect roi(0,0,pt.frame.cols, pt.frame.rows);

    if(event == EVENT_LBUTTONDOWN)
    {
        if(roi.contains(msp) && pt.points.size() < 8)
        {
            pt.points.push_back(msp);
        }
    }
    else if(event == EVENT_RBUTTONDOWN)
    {
        if(pt.points.size() > 0)
        {
            pt.points.pop_back();
        }
    }
}

PerspectiveTransform::PerspectiveTransform(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh(nh)
    , pnh(pnh)
    , winName("Perspective Transform")
{
    std::string source_name;

    pnh.param<std::string>("source_name", source_name, "usb_cam_1/image_raw/compressed");

    subCompImg = nh.subscribe(source_name, 1, &PerspectiveTransform::imageCallback, this);

    points.reserve(8);
    src.reserve(4);
    dst.reserve(4);

    namedWindow(winName, WINDOW_AUTOSIZE);
    setMouseCallback(winName, onMouse, this);
}

void PerspectiveTransform::printParam()
{
    std::cout << "\nfrom : \n" << src << "\n" <<
                 "to : \n" << dst << "\n" <<
                 "warp matrix : \n" << warp << "\n\n";
}

void PerspectiveTransform::imageCallback(const sensor_msgs::CompressedImage::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    frame = cvPtr->image;

    Mat frameClone = frame.clone();

    for(size_t i = 0; i < points.size(); i++)
    {
        Scalar cl;
        if(i < 4) cl = Scalar(100, 100, 255);
        else cl = Scalar(255, 100, 100);
        circle(frameClone, points[i], 4, cl, -1, LINE_AA);
    }

    Mat result = Mat::zeros(frame.size(), CV_8UC3);
    if(points.size() == 8)
    {
        src.assign(points.begin(), points.begin() + 4);
        dst.assign(points.begin() + 4, points.end());

        warp = getPerspectiveTransform(src, dst);
        warpPerspective(frame, result, warp, result.size());
    }

    Mat merged = mergeImage(1, 2, {frameClone, result});
    imshow(winName, merged);
    waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perspective_transform_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    PerspectiveTransform pt(nh, pnh);
    ros::spin();
    pt.printParam();
    return 0;
}
