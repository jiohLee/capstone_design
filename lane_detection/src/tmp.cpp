#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <vector>

const int row__ = 480;
const int col__ = 864;
const int left_high = 365;
const int left_low = 190;

using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tmp_node");
//    Mat binaryImg = imread("/home/a/workspace/my_ws/binaryimg.jpg", IMREAD_GRAYSCALE);
//    imshow("bin", binaryImg);

//    std::vector<Point2f> srcTri({Point2f(left_high, 0), Point2f(col__ - left_high, 0), Point2f(190, row__), Point2f(col__ - left_low, row__)}); // before perspective transform
//    std::vector<Point2f> dstTri({Point2f(left_high - 20, 0), Point2f(col__ - left_high + 20, 0), Point2f(left_high - 20, row__), Point2f(col__ - left_high + 20, row__)});// after perspective transform

//    Mat birdEye;
//    Mat warp_mat = getPerspectiveTransform(srcTri, dstTri);
//    warpPerspective(binaryImg, birdEye, warp_mat, birdEye.size());
////    imshow("bird", birdEye);

//    Mat rdc_birdeye, rdc_roi;
//    rdc_roi = birdEye(Rect(0, birdEye.rows / 2, birdEye.cols , birdEye.rows / 2));
//    reduce(rdc_roi ,rdc_birdeye, 0, REDUCE_SUM, CV_32F);
//    normalize(rdc_birdeye, rdc_birdeye, 0, row__ , NORM_MINMAX);
//    float * rdc_data = rdc_birdeye.ptr<float>(0);

//    // draw reduce image
//    Mat rdc_img = Mat::zeros(birdEye.size(), birdEye.type());
//    for (int c = 0; c < rdc_birdeye.cols; c++)
//    {
//        if ( static_cast<int>(rdc_data[c]) == 0) continue;
//        line(rdc_img, Point(c, rdc_img.rows - 1), Point(c, rdc_img.rows - 1 - static_cast<int>(rdc_data[c])), Scalar(255));
//    }
//    imshow("rdc", rdc_img);

//    const int box_height = row__ / 6;
//    const int box_width = 150;
//    Size box_size(box_width, box_height);

//    std::vector<int> lane_start;
//    lane_start.reserve(3);
//    for (int c = 1; c < rdc_birdeye.cols; ++c)
//    {
//        if (static_cast<int>(rdc_data[c]) == 0) continue;
//        else if (static_cast<int>(rdc_data[c - 1]) == 0) lane_start.emplace_back(c);
//    }

//    int center[3] = {lane_start[0], lane_start[1], lane_start[2]};
//    Mat tmp = birdEye.clone();
//    cvtColor(birdEye, tmp, COLOR_GRAY2BGR);
//    Mat box;
//    for(int i = 0; i < 6; i++)
//    {
//        for(int j = 0; j < 3; j++)
//        {
//            Point left_top(center[j] - box_width / 2, box_height * (5 - i));
//            Rect roi(left_top, box_size);
//            rectangle(tmp, roi, Scalar(0,255,0));
//            box = birdEye(roi);
//            std::string name = format("box%d", j);

//            uchar * boxData;
//            int sum_x = 0;
//            int cnt = 0;
//            for (int r = 0; r < box.rows; r++)
//            {
//                boxData = box.ptr<uchar>(r);
//                for (int c = 0; c < box.cols; c++)
//                {
//                    if(boxData[c] == 0) continue;
//                    else
//                    {
//                        sum_x += c;
//                        cnt++;
//                    }
//                }
//            }
//            Point ct;
//            if(cnt == 0)
//            {
//                ct.x = center[j];
//            }
//            else
//            {
//                ct.x = sum_x / cnt + left_top.x;
//                ct.y = box_height * (5 - i) + box_height / 2;
//                circle(tmp, ct, 2, Scalar(0,255,0), -1);
//            }
//            center[j] = ct.x;
//        }
//        if(i == 3)
//        {
//            Point left_ct((center[0] + center[1]) / 2, box_height * 3 + box_height / 2);
//            Point right_ct((center[2] + center[1]) / 2, box_height * 3 + box_height / 2);
//            circle(tmp, left_ct, 2, Scalar(0,0,255), -1);
//            circle(tmp, right_ct, 2, Scalar(0,0,255), -1);
//        }
//    }

//    imshow("bird", birdEye);
//    imshow("tmp", tmp);

    {
        Mat img(300, 300, 0, Scalar(0));
        Point p1(30, 30);
        Point p2(60, 60);
        Rect roi(p1, p2);
        std::cout << roi.x << std::endl;
        std::cout << roi.y << std::endl;
        std::cout << roi.width << std::endl;
        std::cout << roi.height << std::endl;
//        Mat tmp = img(roi);
//        std::cout << tmp << std::endl;
    }
    waitKey(0);
    return 0;
}
