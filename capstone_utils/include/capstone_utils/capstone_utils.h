#ifndef CAPSTONE_UTILS_H
#define CAPSTONE_UTILS_H

#include <opencv2/opencv.hpp>

using namespace cv;

Mat mergeImage(int rows, int cols, std::vector<Mat> images)
{
    Size refSize = images[0].size();
    for(size_t i = 0; i < images.size(); i++)
    {
        assert(images[i].type() == CV_8UC3 && images[i].size() == refSize);
    }

    Mat merged = Mat::zeros(refSize.height * rows, refSize.width * cols, CV_8UC3);
    for(int r = 0; r < rows; r++)
    {
        for(int c = 0; c < cols; c++)
        {
            images[r * cols + c].copyTo(merged(Rect(Point(c * (refSize.width) , r * (refSize.height)), refSize)));
        }
    }

    return merged;
}

#endif
