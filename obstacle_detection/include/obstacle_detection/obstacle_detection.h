#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt32.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <opencv2/opencv.hpp>

#include <obstacle_msgs/Obstacle.h>

using namespace cv;

class ObstacleDetection
{
public:
    ObstacleDetection(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:

    // ROS Callbacks
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void imgCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);

    // ROS Service
    ros::NodeHandle& nh;
    ros::NodeHandle& pnh;

    ros::Subscriber subScan;
    ros::Subscriber subCompImg;
    ros::Publisher pubPcd;
    ros::Publisher pubPcdFiltered;
    ros::Publisher pubClusters;
    ros::Publisher pubCenteroids;
    ros::Publisher pubObstacles;

    // ROS Messages
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::PointCloud2 cloudFiltered;
    sensor_msgs::PointCloud2 clusters;
    sensor_msgs::PointCloud2 centeroids;
    obstacle_msgs::Obstacle obstacles;

    // ROS Parameter
    double boxHeight;
    double boxWidth;

    int minClusterSize;
    int maxClusterSize;
    double clusterTolerance;

    int hueMax; // hsv color range
    int hueMin;
    int satMax;
    int satMin;
    int valMax;
    int valMin;

    double f_x; // focal length x
    double f_y; // focal length y
    double c_x; // focal center x
    double c_y; // focal center y

    double k_1; // camera distortion coefficient
    double k_2;
    double p_1;
    double p_2;
    double k_3;

    bool showSource;
    bool showBinary;

    // Functions
    void scan2pointCloud(const sensor_msgs::LaserScan& input, pcl::PointCloud<pcl::PointXYZI> & dst);
    void setROI(const pcl::PointCloud<pcl::PointXYZI> & input, pcl::PointCloud<pcl::PointXYZI> & dst);
    int clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters);
    void erodeAndDilate(Mat &input, int shape, Size kSize, int repeat);
    void imageProjection(const pcl::PointXYZI& lPt, Point & iPt);

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

    enum POINT_IDX
    {
        X = 0,
        Y,
        Z
    };

    // Variables
    Mat src;

    int clustersNum;
    pcl::PointCloud<pcl::PointXYZI> pclClusters;
    pcl::PointCloud<pcl::PointXYZI> pclCenteroids;

    Mat RT; // rotation & translation matrix
    Mat CM; // camera intrinsic paramter matrix
    Mat distortion;

    ros::Time timePointPrev;
    double timePointElapsed;
};

#endif // OBSTACLE_DETECTION_H
