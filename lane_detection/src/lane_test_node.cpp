#include <opencv2/opencv.hpp>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

Mat mat;
Mat origin;
std::vector<Point2f> srcTri;
std::vector<Point2f> dstTri;

void onMouse(int event, int x, int y, int flags, void* userdata)
{
    const int perspectivePointNum = 4;
    const int perspectivePointRadius = 5;

    if (event == EVENT_LBUTTONDOWN)
    {
        if (srcTri.size() < perspectivePointNum)
        {
            srcTri.emplace_back(static_cast<float>(x), static_cast<float>(y));
            circle(mat, Point(x, y), perspectivePointRadius, Scalar(255, 0, 0), FILLED);
        }
        else
        {
            std::cout << "wrong transform point number press R button" << std::endl;
        }
    }
    else if (event == EVENT_RBUTTONDOWN)
    {
        if (srcTri.size() == perspectivePointNum && dstTri.size() < perspectivePointNum)
        {
            dstTri.emplace_back(static_cast<float>(x), static_cast<float>(y));
            circle(mat, Point(x, y), perspectivePointRadius, Scalar(0, 0, 255), FILLED);
            if (dstTri.size() == perspectivePointNum)
            {
                for (const auto& p : srcTri)
                {
                    std::cout << p << std::endl;
                }std::cout << std::endl;

                for (const auto& p : dstTri)
                {
                    std::cout << p << std::endl;
                }std::cout << std::endl;

                Mat warp_mat = getPerspectiveTransform(srcTri, dstTri);
                Mat dst = Mat::zeros(mat.size(), mat.type());
                warpPerspective(origin, dst, warp_mat, dst.size());
                imshow("result", dst);
            }
        }
        else
        {
            std::cout << "wrong transform point number press L button" << std::endl;
        }
    }
    imshow("origin", mat);
}

void onChange1(int , void*) { }
void onChange2(int , void*) { }
void onChange3(int , void*) { }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Lane_test_node");
    Mat src = imread("/home/a/line.jpg", IMREAD_COLOR);
    ros::NodeHandle pnh("~");
    CV_Assert(src.data);
    int num = 0;
    pnh.param<int>("test_select", num, 0);

    if (num == 0)
    {
        /*
         * perspective transform
        */
        mat = src.clone();
        origin = src.clone();
        imshow("origin", src);
        setMouseCallback("origin", onMouse);
    }
    else if (num == 1)
    {
        /*
         * channel transform
        */
        imshow("origin", src);
        Mat dst;
        cvtColor(src,dst,COLOR_BGR2HSV);
        Mat dst1;
        Mat dst2;
        Mat bin;
        int hue1_high = 0;
        int hue1_low = 0;
        int hue2_high = 0;
        int hue2_low = 0;
        int sat = 0;
        int val = 0;
        const int rg = 10;
        namedWindow("dst1", WINDOW_AUTOSIZE);
        createTrackbar("hue1_high", "dst1", &hue1_high, 179, onChange1);
        createTrackbar("hue1_low", "dst1", &hue1_low, 179, onChange1);
        createTrackbar("hue2_high", "dst1", &hue2_high, 179, onChange1);
        createTrackbar("hue2_low", "dst1", &hue2_low, 179, onChange1);
        createTrackbar("sat", "dst1", &sat, 255, onChange2);
        createTrackbar("val", "dst1", &val, 255, onChange3);

        while (true)
        {
            inRange(dst, Scalar(hue1_low, sat, val), Scalar(hue1_high, 255, 255),dst1);
            inRange(dst, Scalar(hue2_low, sat, val), Scalar(hue2_high, 255, 255),dst2);
            bin = dst1 + dst2;
            imshow("bin", bin);
            if (waitKey(30) == 27) break;
        }
    }
    else if (num == 2)
    {

    }

    waitKey(0);
    return 0;
}
