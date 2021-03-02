#include <iostream>
#include <vector>

#include <cv_bridge/cv_bridge.h>

#include "lane_detection/lane_detection.h"

constexpr int row__ = 480;
constexpr int col__ = 864;
constexpr int left_high = 230;
constexpr int left_low = 100;

LaneDetection::LaneDetection(ros::NodeHandle & nh, ros::NodeHandle & pnh)
    : nh(nh)
    , pnh(pnh)
    , srcTri({Point2f(left_high, 0), Point2f(col__ - left_high, 0), Point2f(left_low, row__), Point2f(col__ - left_low, row__)}) // before perspective transform
    , dstTri({Point2f(left_high, 0), Point2f(col__ - left_high, 0), Point2f(left_high, row__), Point2f(col__ - left_high, row__)})// after perspective transform
{
    subCompImg = nh.subscribe("usb_cam_1/image_raw/compressed", 1, &LaneDetection::imgCallback, this);
    pubTargetSteer = nh.advertise<std_msgs::Float64>("/lane_detection/target_steer", 1);
    pubOnLane = nh.advertise<std_msgs::String>("/lane_detection/on_lane", 1);

    // Save Last Image For Transformation
    pnh.param<bool>("save_last_img", bSaveLastImage, false);

    // HSV Color Ranges
    pnh.param<int>("red_hue_1_max", redHMax1, 10);
    pnh.param<int>("red_hue_1_min", redHMin1, 0);
    pnh.param<int>("red_hue_2_max", redHMax2, 179);
    pnh.param<int>("red_hue_2_min", redHMin2, 159);
    pnh.param<int>("yellow_hue_max", yellowHMax, 50);
    pnh.param<int>("yellow_hue_min", yellowHMin, 20);
    pnh.param<int>("sat_max", satMax, 255);
    pnh.param<int>("sat_min", satMin, 40);
    pnh.param<int>("val_max", valMax, 255);
    pnh.param<int>("val_min", valMin, 120);

    // Sliding Window Params
    pnh.param<int>("window_width", windowWidth, 3);
    pnh.param<int>("window_num", windowNum, 3);
    pnh.param<int>("target_window_height", targetWindowHeight, 3);

    pnh.param<bool>("show_source", showSource, true);
    pnh.param<bool>("show_sliding_window", showSlidingWindow, true);

    centeroidsRed.resize(static_cast<size_t>(windowNum));
    centeroidsYellow.resize(static_cast<size_t>(windowNum));

    // Init Time Point
    timePointPrev = ros::Time::now();
}

void LaneDetection::saveLastImage()
{
    if (bSaveLastImage)
    {
        imwrite("/home/a/line.jpg", src);
        imwrite("/home/a/topView.jpg", topView);
        std::cout << "\n\nIMAGE SAVED!!\n\n";
    }
}

void LaneDetection::imgCallback(const sensor_msgs::CompressedImage::ConstPtr & msg)
{
    // convert sensor_msgs to cv::Mat
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    src = cvPtr->image;

    /*
     * get perspective transform
    */
    Mat warp_mat = getPerspectiveTransform(srcTri, dstTri);
    warpPerspective(src, topView, warp_mat, topView.size());

    /*
     * get binary image
    */
    Mat hsv;
    cvtColor(topView, hsv, COLOR_BGR2HSV);

    // red lane binary
    Mat dstRed1 = Mat::zeros(src.size(), src.type());
    Mat dstRed2 = Mat::zeros(src.size(), src.type());
    inRange(hsv, Scalar(redHMin1, satMin, valMin), Scalar(redHMax1, satMax, valMax),dstRed1);
    inRange(hsv, Scalar(redHMin2, satMin, valMin), Scalar(redHMax2, satMax, valMax),dstRed2);
    topViewBinRed = dstRed1 + dstRed2;

    // yellow lane binary
    inRange(hsv, Scalar(yellowHMin, satMin, valMin), Scalar(yellowHMax, satMax, valMax),topViewBinYellow);

    // remove salt noise
    erodeAndDilate(topViewBinRed, MorphShapes::MORPH_RECT, Size(3,3), 3);
    erodeAndDilate(topViewBinYellow, MorphShapes::MORPH_RECT, Size(3,3), 3);

    /*
     * sliding window method
    */

    // get centeroids of both lane;
//    std::vector<Point> centeroidsYellow;
//    std::vector<Point> centeroidsRed;
    getSlidingWindow(topViewBinYellow, centeroidsYellow, windowWidth, windowNum);
    getSlidingWindow(topViewBinRed, centeroidsRed, windowWidth, windowNum);

    // publish target steer and on lane
    Point targetWayPoint;
    targetWayPoint.x = std::abs((centeroidsRed[static_cast<size_t>(targetWindowHeight)].x + centeroidsYellow[static_cast<size_t>(targetWindowHeight)].x)/ 2);
    targetWayPoint.y = std::abs(centeroidsRed[static_cast<size_t>(targetWindowHeight)].y);

    targetSteer.data = std::atan2((topView.cols / 2) - targetWayPoint.x, topView.rows - targetWayPoint.y);
    pubTargetSteer.publish(targetSteer);

    if (centeroidsRed[static_cast<size_t>(targetWindowHeight)].x - centeroidsYellow[static_cast<size_t>(targetWindowHeight)].x > 0)
    {
        onLane.data = "ON_LANE_RIGHT";
    }
    else
    {
        onLane.data = "ON_LANE_LEFT";
    }
    pubOnLane.publish(onLane);

    // draw sliding window
    Mat topViewMerged;
    Mat slidingWindowImg;
    bitwise_or(topViewBinRed, topViewBinYellow, topViewMerged);
    cvtColor(topViewMerged, slidingWindowImg, COLOR_GRAY2BGR);
    drawSlidingWindow(slidingWindowImg, centeroidsRed, windowWidth);
    drawSlidingWindow(slidingWindowImg, centeroidsYellow, windowWidth);

    line(slidingWindowImg, Point(slidingWindowImg.cols / 2 - 200, targetWayPoint.y), Point(slidingWindowImg.cols / 2 + 200, targetWayPoint.y), Scalar(0, 255, 255), 1);
    line(slidingWindowImg, Point(slidingWindowImg.cols / 2, targetWayPoint.y - 20), Point(slidingWindowImg.cols / 2, targetWayPoint.y + 20), Scalar(0, 255, 255), 1);
    line(slidingWindowImg, Point(slidingWindowImg.cols / 2 - 150, targetWayPoint.y - 20), Point(slidingWindowImg.cols / 2 - 150, targetWayPoint.y + 20), Scalar(0, 255, 255), 1);
    line(slidingWindowImg, Point(slidingWindowImg.cols / 2 + 150, targetWayPoint.y - 20), Point(slidingWindowImg.cols / 2 + 150, targetWayPoint.y + 20), Scalar(0, 255, 255), 1);
    line(slidingWindowImg, Point(slidingWindowImg.cols / 2 - 200, targetWayPoint.y - 20), Point(slidingWindowImg.cols / 2 - 200, targetWayPoint.y + 20), Scalar(0, 255, 255), 1);
    line(slidingWindowImg, Point(slidingWindowImg.cols / 2 + 200, targetWayPoint.y - 20), Point(slidingWindowImg.cols / 2 + 200, targetWayPoint.y + 20), Scalar(0, 255, 255), 1);
    circle(slidingWindowImg, targetWayPoint, 2, Scalar(0,0,255), -1);

    if(showSource) imshow("Source", src);
    if(showSlidingWindow) imshow("Sliding Window", slidingWindowImg);

    timePointElapsed = ros::Time::now().toSec() - timePointPrev.toSec();
    timePointPrev = ros::Time::now();

    ROS_INFO("LANE : %s", onLane.data.c_str());
    ROS_INFO("TARGET STEER : %lf", targetSteer.data);
    ROS_INFO("TIME ELAPSED : %lf [ms]\n", timePointElapsed * 1000.0);

    waitKey(1);
}

void LaneDetection::getSlidingWindow(Mat &input, std::vector<Point> &centeroids, int windowWidth, int windowNum)
{
    Mat labels;
    Mat stats;
    Mat tmpCenteroids;
    connectedComponentsWithStats(input, labels, stats, tmpCenteroids, 8);

    int lowerY = 0;
    Point start;
    int idx = 0;
    for ( int i = 1; i < stats.rows; i++)
    {
        int * statsPtr = stats.ptr<int>(i);
        int leftLowY = static_cast<int>(statsPtr[LEFT_TOP_Y] + statsPtr[HEIGHT]);
        int leftLowX = static_cast<int>(statsPtr[LEFT_TOP_X]);
        if(lowerY < leftLowY)
        {
            start.x = leftLowX;
            start.y = leftLowY;
            idx = i;
            lowerY = leftLowY;
        }
    }


    uchar* inputPtr = input.ptr<uchar>(start.y - 1);
    int startX = 0;
    for (int i = start.x - 1; i < input.cols ; i++)
    {
        if(inputPtr[i] > 0)
        {
            startX = i;
            break;
        }
    }

//    centeroids.clear();
//    centeroids.reserve(static_cast<size_t>(tmpCenteroids.rows - 1));
    int windowHeight = input.rows / windowNum;
    Point lt(startX, windowHeight * (windowNum - 1));
    Size windowSize(windowWidth, windowHeight);
    Rect roi(lt, windowSize);
    for (size_t i = 0; i < static_cast<size_t>(windowNum); i++)
    {

        if( roi.x < 0) roi.x = 0;
        if(roi.x + roi.width > input.cols) roi.x = input.cols - roi.width;
        int centerX = roi.width / 2;
        int centerY = roi.height / 2;
        Mat window = input(roi);

        uchar* windowPtr = nullptr;
        int sum = 0;
        int cnt = 0;

        for(int r = 0; r < window.rows; r++)
        {
            windowPtr = window.ptr<uchar>(r);
            for (int c = 0; c < window.cols; c++)
            {
                if(windowPtr[c] == 0) continue;
                else
                {
                    sum += c;
                    cnt++;
                }
            }
        }

        if(cnt == 0)
        {
            if(i > 1)
            {
                double pre = (centeroids[i - 1].x + centeroids[i - 2].x) / 2;
                centerX = static_cast<int>(((centeroids[i].x + pre) / 2) - roi.x);
            }
        }
        else
        {
            centerX = sum / cnt;
        }

        centeroids[i] = Point(centerX + roi.x, centerY + roi.y);

        roi.x += centerX - roi.width / 2;
        roi.y -= roi.height;

    }
}

void LaneDetection::drawSlidingWindow(Mat &input, std::vector<Point>& centeroids, int windowWidth)
{
    int windowNum = static_cast<int>(centeroids.size());
    int windowHeight = input.rows / windowNum;
    for (size_t i = 0; i < static_cast<size_t>(windowNum); i++)
    {
        rectangle(input, Rect(centeroids[i] - Point(windowWidth / 2, windowHeight / 2), Size(windowWidth, windowHeight)), Scalar(0,255,0));
        circle(input, centeroids[i], 2, Scalar(0,255,0), -1);
    }
}

void LaneDetection::erodeAndDilate(Mat &input, int shape, Size kSize, int repeat)
{
    Mat element = getStructuringElement(shape, kSize);
    Mat morph = input;

    for (int i = 0; i < repeat; i++)
    {
        erode(morph, morph, element);
    }

    for (int i = 0; i < repeat; i++)
    {
        dilate(morph, morph, element);
    }
}
