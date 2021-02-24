#include <math.h>

#include "obstacle_detection/obstacle_detection.h"

template <typename T>
constexpr T rad2deg(T x)
{
    return x * static_cast<T>(180) / M_PI;
}

template <typename T>
constexpr T deg2rad(T x)
{
    return static_cast<T>(static_cast<double>(x) * M_PI / 180.0);
}


ObstacleDetection::ObstacleDetection(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    :nh(nh)
    ,pnh(pnh)
{
    subScan = nh.subscribe("/scan", 1, &ObstacleDetection::laserScanCallback, this);
    pubPcd = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1);
    pubPcdFiltered = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_filtered", 1);
    pubClusters = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_cluster", 1);
    pubCenteroids = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_cluster_centeroids", 1);

    pnh.param<double>("box_height", boxHeight, 2);
    pnh.param<double>("box_width", boxWidth, 2);

    pnh.param<int>("min_cluster_size", minClusterSize, 2);
    pnh.param<int>("max_cluster_size", maxClusterSize, 400);
    pnh.param<double>("cluster_tolerance", clusterTolerance, 0.1);
}

void ObstacleDetection::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    const sensor_msgs::LaserScan & scan = *msg;

    // to pcl point cloud
    pcl::PointCloud<pcl::PointXYZI> pclCloud;
    scan2pointCloud(scan, pclCloud);
    pcl::toROSMsg(pclCloud, cloud);
    pubPcd.publish(cloud); // publish raw point cloud

    // set ROI of pclCloud;
    pcl::PointCloud<pcl::PointXYZI> pclCloudFiltered;
    pclCloudFiltered.header = pclCloud.header;
    setROI(pclCloud, pclCloudFiltered);

    pcl::toROSMsg(pclCloudFiltered, cloudFiltered);
    pubPcdFiltered.publish(cloudFiltered); // publish roi point cloud

    // clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>> pclClusterVector;
    int clusterNum = 0;
    if (pclCloudFiltered.size() != 0)
    {
        clusterNum = clustering(pclCloudFiltered.makeShared(), pclClusterVector);
    }

    // merge pcl cluster vector to single cloud
    pcl::PointCloud<pcl::PointXYZI> pclClusters;
    pclClusters.header = pclCloudFiltered.header;
    for (size_t i = 0; i < pclClusterVector.size(); i++)
    {
        const pcl::PointCloud<pcl::PointXYZI> & cluster = pclClusterVector[i];
        for (size_t j = 0; j < cluster.size(); ++j)
        {
            pclClusters.push_back(cluster[j]);
        }
    }
    pcl::toROSMsg(pclClusters, clusters);
    pubClusters.publish(clusters); // publish clusterd point cloud

    // get centeroid of point cloud;
    pcl::PointCloud<pcl::PointXYZI> pclCenteroids;
    pclCenteroids.header = pclClusters.header;

    for (size_t i = 0; i < pclClusterVector.size(); i++)
    {
        const pcl::PointCloud<pcl::PointXYZI> & pclCluster = pclClusterVector[i];
        pcl::PointXYZI pclCenteroid;
        pclCenteroid.intensity = pclCluster[0].intensity;
        for (size_t j = 0; j < pclCluster.size(); j++)
        {
            pclCenteroid.x += pclCluster[j]._PointXYZI::x;
            pclCenteroid.y += pclCluster[j]._PointXYZI::y;
        }
        pclCenteroid.x /= pclCluster.size();
        pclCenteroid.y /= pclCluster.size();
        pclCenteroids.push_back(pclCenteroid);
    }

    pcl::toROSMsg(pclCenteroids, centeroids);
    pubCenteroids.publish(centeroids); // publish centeroids of clusters

    ROS_INFO("seq : %d", scan.header.seq);
    ROS_INFO("cluster size : %d\n", static_cast<int>(clusterNum));
}

void ObstacleDetection::scan2pointCloud(const sensor_msgs::LaserScan& input, pcl::PointCloud<pcl::PointXYZI> & dst)
{
    if (input.ranges.size() == 0) return;

    dst.clear();
    dst.width = static_cast<uint32_t>(input.ranges.size());
    dst.height = 1;
    dst.header.seq = input.header.seq;
    dst.header.frame_id = input.header.frame_id;

    pcl::PointXYZI pt;

    for (size_t i = 0; i < input.ranges.size(); ++i)
    {
        float radAngle = i * input.angle_increment;

        pt.x = cos(radAngle) * input.ranges[i];
        pt.y = sin(radAngle) * input.ranges[i];
        pt.z = 0;
        pt.intensity = input.intensities[i];
        dst.push_back(pt);
    }
}

void ObstacleDetection::setROI(const pcl::PointCloud<pcl::PointXYZI> &input, pcl::PointCloud<pcl::PointXYZI> &dst)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputPtr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass;

    float xRange = static_cast<float>(boxHeight) / 2.0f;
    float yRange = static_cast<float>(boxWidth) / 2.0f;

    pass.setInputCloud(input.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-yRange, yRange);
    pass.setFilterLimitsNegative(false);
    pass.filter(*inputPtr);

    pass.setInputCloud(inputPtr);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.075f, 0.075f);
    pass.setFilterLimitsNegative(true);
    pass.filter(*inputPtr);

    pass.setInputCloud(inputPtr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-xRange, xRange);
    pass.setFilterLimitsNegative(false);
    pass.filter(*inputPtr);
    dst += *inputPtr;

    pass.setInputCloud(input.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, xRange);
    pass.setFilterLimitsNegative(false);
    pass.filter(*inputPtr);

    pass.setInputCloud(inputPtr);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.075f, 0.075f);
    pass.setFilterLimitsNegative(false);
    pass.filter(*inputPtr);
    dst += *inputPtr;
}

int ObstacleDetection::clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::vector<pcl::PointCloud<pcl::PointXYZI>> &clusters)
{
    clusters.clear();
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(input);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    std::vector<pcl::PointIndices> clusterIndices;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(clusterIndices);

    clusters.reserve(clusterIndices.size());
    int id = 0;

    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it < clusterIndices.end(); it++)
    {
        pcl::PointCloud<pcl::PointXYZI> cluster;
        cluster.reserve(it->indices.size());
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit < it->indices.end(); pit++)
        {
            pcl::PointXYZI pt = input->at(static_cast<size_t>(*pit));
            pt.intensity = clusters.size();
            cluster.push_back(pt);
        }
        clusters.emplace_back(std::move(cluster));
        id++;
    }
    return id;
}














