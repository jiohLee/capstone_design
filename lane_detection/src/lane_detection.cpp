#include <iostream>
#include <vector>

#include <cv_bridge/cv_bridge.h>

#include "lane_detection/lane_detection.h"

const int row__ = 480;
const int col__ = 864;
const int left_high = 235;
const int left_low = 105;

LaneDetection::LaneDetection(ros::NodeHandle & nh, ros::NodeHandle & pnh)
    :srcTri({Point2f(left_high, 0), Point2f(col__ - left_high, 0), Point2f(left_low, row__), Point2f(col__ - left_low, row__)}) // before perspective transform
    ,dstTri({Point2f(left_high, 0), Point2f(col__ - left_high, 0), Point2f(left_high, row__), Point2f(col__ - left_high, row__)})// after perspective transform
{
    subCompImg = nh.subscribe("usb_cam/image_raw/compressed", 1, &LaneDetection::imgCallback, this);
    pubPoint = nh.advertise<geometry_msgs::Point>("/waypoint", 1);
    namedWindow("src", WINDOW_AUTOSIZE);

    // save image
    pnh.param<bool>("save_last_img", save_last_image, false);

    // get color ny HSV
    pnh.param<int>("hue_1_max", hue_1_max, 10);
    pnh.param<int>("hue_1_min", hue_1_min, 0);
    pnh.param<int>("hue_2_max", hue_2_max, 179);
    pnh.param<int>("hue_2_min", hue_2_min, 159);
    pnh.param<int>("sat_max", sat_max, 255);
    pnh.param<int>("sat_min", sat_min, 40);
    pnh.param<int>("val_max", val_max, 255);
    pnh.param<int>("val_min", val_min, 120);

    // imshow srouce & retult;
    pnh.param<bool>("show_source", show_source, true);
    pnh.param<bool>("show_reduced", show_reduced, true);
    pnh.param<bool>("show_sliding_window", show_sliding_window, true);

    // waypoint
    pnh.param<int>("waypoint_height", waypoint_height, 3);
}

void LaneDetection::saveLastImage()
{
    if (save_last_image)
    {
        imwrite("/home/a/workspace/my_ws/line.jpg", src);
        imwrite("/home/a/workspace/my_ws/birdeye.jpg", birdEye);
        imwrite("/home/a/workspace/my_ws/binaryimg.jpg", binaryImg);
        std::cout << "\n\nIMAGE SAVED!!\n\n";
    }
}

void LaneDetection::imgCallback(const sensor_msgs::CompressedImage::ConstPtr & msg)
{
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    src = cvPtr->image;
    if (show_source) imshow("src", src);

    Mat warp_mat = getPerspectiveTransform(srcTri, dstTri);
    Mat dst1 = Mat::zeros(src.size(), src.type());
    Mat dst2 = Mat::zeros(src.size(), src.type());
    Mat dst_lane = Mat::zeros(src.size(), src.type());
    Mat hsv;

    // get binary image
    cvtColor(src, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(hue_1_min, sat_min, val_min), Scalar(hue_1_max, sat_max, val_max),dst1);
    inRange(hsv, Scalar(hue_2_min, sat_min, val_min), Scalar(hue_2_max, sat_max, val_max),dst2);
    binaryImg = dst1 + dst2;

    // get bird-eye-view
    warpPerspective(binaryImg, birdEye, warp_mat, birdEye.size());

    // get reduced;
    Mat rdc_birdeye, rdc_roi;
    rdc_roi = birdEye(Rect(0, birdEye.rows * 2 / 3, birdEye.cols , birdEye.rows / 3));
    reduce(rdc_roi ,rdc_birdeye, 0, REDUCE_SUM, CV_32S);
    normalize(rdc_birdeye, rdc_birdeye, 0, row__ , NORM_MINMAX);
    int * rdc_data = rdc_birdeye.ptr<int>(0);

    // reduce img
    if (show_reduced)
    {
        Mat rdc_img = Mat::zeros(birdEye.size(), birdEye.type());
        for (int c = 0; c < rdc_birdeye.cols; c++)
        {
            if ( static_cast<int>(rdc_data[c]) == 0) continue;
            line(rdc_img, Point(c, rdc_img.rows - 1), Point(c, rdc_img.rows - 1 - rdc_data[c]), Scalar(255));
        }
        imshow("rdc", rdc_img);
    }

    // get lane x position using row-reduced data
    std::vector<int> lane_start{0,rdc_birdeye.cols - 1};
    for (int c = 1; c < rdc_birdeye.cols; ++c)
    {
        if(rdc_data[c] != 0)
        {
            lane_start[0] = c;
            break;
        }
    }
    for (int c = 1; c < rdc_birdeye.cols; ++c)
    {
        if(rdc_data[rdc_birdeye.cols - c] != 0)
        {
            lane_start[1] = rdc_birdeye.cols - c;
            break;
        }
    }

    // sliding window
    const int box_height = row__ / 6;
    const int box_width = 200;
    Size box_size(box_width, box_height);
    Mat box;

    // tmp to visualize sliding window
    Mat tmp;
    if(show_sliding_window)
    {
        tmp = birdEye.clone();
        cvtColor(birdEye, tmp, COLOR_GRAY2BGR);
    }

    int center[2] = {lane_start[0], lane_start[1]};
    int left_lane[6] = {0,0,0,0,0,0};
    int right_lane[6] = {0,0,0,0,0,0};

    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 2; j++)
        {
            Point left_top(center[j] - box_width / 2, box_height * (5 - i));
            Point right_bottom(center[j] + box_width / 2, box_height * (6 - i));
            Rect roi(left_top, box_size);

            if(roi.x < 0) roi.x = 0;
            if(roi.width + roi.x > birdEye.cols) roi.x = birdEye.cols - box_width;
            box = birdEye(roi);

            uchar * boxData;
            int sum_x = 0;
            int cnt = 0;

            for (int r = 0; r < box.rows; r++)
            {
                boxData = box.ptr<uchar>(r);
                for (int c = 0; c < box.cols; c++)
                {
                    if(boxData[c] == 0) continue;
                    else
                    {
                        sum_x += c;
                        cnt++;
                    }
                }
            }

            Point ct;
            if(cnt == 0)
            {
                ct.x = center[j];
                if(i > 1)
                {
                    if(j == 0)
                    {
                        ct.x += left_lane[i - 1] - left_lane[i - 2] - 50;
                    }
                    else if(j == 1)
                    {
                        ct.x += right_lane[i - 1] - right_lane[i - 2] + 50;
                    }
                }
            }
            else
            {
                ct.x = sum_x / cnt + left_top.x;
            }
            ct.y = box_height * (5 - i) + box_height / 2;
            center[j] = ct.x;

            if ( j == 0)
            {
                left_lane[i] = ct.x;
            }
            else if (j == 1)
            {
                right_lane[i] = ct.x;
            }
        }
    }

    Point waypoint((left_lane[waypoint_height] + right_lane[waypoint_height]) / 2, box_height * (6 - waypoint_height) - box_height / 2);
    pt.x = birdEye.rows - waypoint.y;
    pt.y = birdEye.cols / 2 - waypoint.x;
    pubPoint.publish(pt);

    if(show_sliding_window)
    {
        for (int i = 0; i < 6 ; i++)
        {
            Rect window_left(left_lane[i] - box_width / 2, box_height * (5 - i), box_width, box_height );
            Rect window_right(right_lane[i] - box_width / 2, box_height * (5 - i), box_width, box_height );
            rectangle(tmp, window_left, Scalar(0,255,0));
            rectangle(tmp, window_right, Scalar(0,255,0));
            circle(tmp, Point(left_lane[i], box_height * (5 - i) + box_height / 2), 2, Scalar(0,255,0), -1);
            circle(tmp, Point(right_lane[i], box_height * (5 - i) + box_height / 2), 2, Scalar(0,255,0), -1);
        }

        line(tmp, Point(tmp.cols / 2 - 200, waypoint.y), Point(tmp.cols / 2 + 200, waypoint.y), Scalar(0, 255, 255), 1);
        line(tmp, Point(tmp.cols / 2, waypoint.y - 20), Point(tmp.cols / 2, waypoint.y + 20), Scalar(0, 255, 255), 1);
        line(tmp, Point(tmp.cols / 2 - 150, waypoint.y - 20), Point(tmp.cols / 2 - 150, waypoint.y + 20), Scalar(0, 255, 255), 1);
        line(tmp, Point(tmp.cols / 2 + 150, waypoint.y - 20), Point(tmp.cols / 2 + 150, waypoint.y + 20), Scalar(0, 255, 255), 1);
        line(tmp, Point(tmp.cols / 2 - 200, waypoint.y - 20), Point(tmp.cols / 2 - 200, waypoint.y + 20), Scalar(0, 255, 255), 1);
        line(tmp, Point(tmp.cols / 2 + 200, waypoint.y - 20), Point(tmp.cols / 2 + 200, waypoint.y + 20), Scalar(0, 255, 255), 1);
        circle(tmp, waypoint, 2, Scalar(0,0,255), -1);

        imshow("tmp", tmp);
    }

    waitKey(1);
}
