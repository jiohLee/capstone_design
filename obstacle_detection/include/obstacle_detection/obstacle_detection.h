#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

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

//#include <pcl_ros/point_cloud.h>
//#include <pcl/io/pcd_io.h>

//#include <pcl/filters/extract_indices.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <pcl/PointIndices.h>
//#include <pcl/point_traits.h>
//#include <pcl/common/impl/centroid.hpp>
//#include <pcl/common/impl/common.hpp>
//#include <pcl/pcl_base.h>

class ObstacleDetection
{
public:
    ObstacleDetection(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:

    // ros service
    ros::NodeHandle& nh;
    ros::NodeHandle& pnh;

    ros::Publisher pubPcd;
    ros::Publisher pubPcdFiltered;
    ros::Publisher pubClusters;
    ros::Publisher pubCenteroids;
    ros::Subscriber subScan;

    // ros messages
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::PointCloud2 cloudFiltered;
    sensor_msgs::PointCloud2 clusters;
    sensor_msgs::PointCloud2 centeroids;

    // ros parameter
    double boxHeight;
    double boxWidth;

    int minClusterSize;
    int maxClusterSize;
    double clusterTolerance;

    double lamda;

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void scan2pointCloud(const sensor_msgs::LaserScan& input, pcl::PointCloud<pcl::PointXYZI> & dst);
    void setROI(const pcl::PointCloud<pcl::PointXYZI> & input, pcl::PointCloud<pcl::PointXYZI> & dst);
    int clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters);

    // ros msgs
};

#endif // OBSTACLE_DETECTION_H
