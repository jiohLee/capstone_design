#include <iostream>
#include <vector>

#include <cv_bridge/cv_bridge.h>

#include "lane_detection/lane_detection.h"

LaneDetection::LaneDetection(ros::NodeHandle & nh, ros::NodeHandle & pnh)
    : nh(nh)
    , pnh(pnh)
{
    subCompImg = nh.subscribe("usb_cam_1/image_raw/compressed", 1, &LaneDetection::imgCallback, this);
    pubLaneCenterPoint = nh.advertise<geometry_msgs::Point>("/lane_detection/lane_center_point", 1);
    pubOnLane = nh.advertise<std_msgs::String>("/lane_detection/on_lane", 1);

    // Perspective Transform Params
    pnh.param<int>("left_high", leftHigh, 303);
    pnh.param<int>("left_low", leftLow, 151);
    srcTri.assign({Point2f(leftHigh, 0), Point2f(col__ - leftHigh, 0), Point2f(leftLow, row__), Point2f(col__ - leftLow, row__)});
    dstTri.assign({Point2f(leftHigh, 0), Point2f(col__ - leftHigh, 0), Point2f(leftHigh, row__), Point2f(col__ - leftHigh, row__)});

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
    getSlidingWindow(topViewBinYellow, centeroidsYellow, windowWidth, windowNum);
    getSlidingWindow(topViewBinRed, centeroidsRed, windowWidth, windowNum);

    // publish target steer and on lane
    Point targetWayPoint;
    targetWayPoint.x = std::abs((centeroidsRed[static_cast<size_t>(targetWindowHeight)].x + centeroidsYellow[static_cast<size_t>(targetWindowHeight)].x)/ 2);
    targetWayPoint.y = std::abs(centeroidsRed[static_cast<size_t>(targetWindowHeight)].y);

    laneCenterPoint.x = (topView.rows - targetWayPoint.y) / px_per_m;
    laneCenterPoint.y = ((topView.cols / 2) - targetWayPoint.x) / px_per_m;

    float alpha = std::atan2(-laneCenterPoint.y, laneCenterPoint.x);
    float lineTangent = alpha;
    float a = std::cos(lineTangent);
    float b = std::sin(lineTangent);
    pubLaneCenterPoint.publish(laneCenterPoint);

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

    // draw reference line
    line(slidingWindowImg, Point(slidingWindowImg.cols / 2 - 0.3 * px_per_m, targetWayPoint.y), Point(slidingWindowImg.cols / 2 + 0.3 * px_per_m, targetWayPoint.y), Scalar(0, 255, 255), 1);
    line(slidingWindowImg, Point(slidingWindowImg.cols / 2, targetWayPoint.y - 20), Point(slidingWindowImg.cols / 2, targetWayPoint.y + 20), Scalar(0, 255, 255), 1);
    line(slidingWindowImg, Point(slidingWindowImg.cols / 2 - 0.25 * px_per_m, targetWayPoint.y - 20), Point(slidingWindowImg.cols / 2 - 0.25 * px_per_m, targetWayPoint.y + 20), Scalar(0, 255, 255), 1);
    line(slidingWindowImg, Point(slidingWindowImg.cols / 2 + 0.25 * px_per_m, targetWayPoint.y - 20), Point(slidingWindowImg.cols / 2 + 0.25 * px_per_m, targetWayPoint.y + 20), Scalar(0, 255, 255), 1);
    line(slidingWindowImg, Point(slidingWindowImg.cols / 2 - 0.3 * px_per_m, targetWayPoint.y - 20), Point(slidingWindowImg.cols / 2 - 0.3 * px_per_m, targetWayPoint.y + 20), Scalar(0, 255, 255), 1);
    line(slidingWindowImg, Point(slidingWindowImg.cols / 2 + 0.3 * px_per_m, targetWayPoint.y - 20), Point(slidingWindowImg.cols / 2 + 0.3 * px_per_m, targetWayPoint.y + 20), Scalar(0, 255, 255), 1);
    circle(slidingWindowImg, targetWayPoint, 2, Scalar(0,0,255), -1);

    if(showSource) imshow("Source", src);
    if(showSlidingWindow) imshow("Sliding Window", slidingWindowImg);

    timePointElapsed = ros::Time::now().toSec() - timePointPrev.toSec();
    timePointPrev = ros::Time::now();

    ROS_INFO("LANE : %s", onLane.data.c_str());
    ROS_INFO("TARGET STEER : %lf", alpha);
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
