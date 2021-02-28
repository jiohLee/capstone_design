#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>

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

//#include <pcl_ros/point_cloud.h>
//#include <pcl/io/pcd_io.h>

//#include <pcl/filters/extract_indices.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <pcl/PointIndices.h>
//#include <pcl/point_traits.h>
//#include <pcl/common/impl/centroid.hpp>
//#include <pcl/common/impl/common.hpp>
//#include <pcl/pcl_base.h>

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

    // ROS Messages
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::PointCloud2 cloudFiltered;
    sensor_msgs::PointCloud2 clusters;
    sensor_msgs::PointCloud2 centeroids;

    // ROS Parameter
    double boxHeight;
    double boxWidth;

    int minClusterSize;
    int maxClusterSize;
    double clusterTolerance;

    int hueMax;
    int hueMin;
    int satMax;
    int satMin;
    int valMax;
    int valMin;

    double camAngleLR;     // deg
    double camAngleUD;     // deg
    double xDistCamLidar;  // m
    double zDistCamLidar;  // m

    bool showSource;

    // Functions
    void scan2pointCloud(const sensor_msgs::LaserScan& input, pcl::PointCloud<pcl::PointXYZI> & dst);
    void setROI(const pcl::PointCloud<pcl::PointXYZI> & input, pcl::PointCloud<pcl::PointXYZI> & dst);
    int clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters);

    // Variables
//    Mat src;

    int clustersNum;
    pcl::PointCloud<pcl::PointXYZI> pclClusters;
    pcl::PointCloud<pcl::PointXYZI> pclCenteroids;

    ros::Time timePointPrev;
    double timePointElapsed;
};

#endif // OBSTACLE_DETECTION_H
