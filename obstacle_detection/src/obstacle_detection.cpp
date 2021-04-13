#include <math.h>

#include <cv_bridge/cv_bridge.h>

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
    subCompImg = nh.subscribe("/usb_cam_2/image_raw/compressed",1 , &ObstacleDetection::imgCallback, this);
    pubPcd = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_detection/point_cloud", 1);
    pubPcdFiltered = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_detection/point_cloud_filtered", 1);
    pubClusters = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_detection/point_cloud_cluster", 1);
    pubCenteroids = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_detection/point_cloud_cluster_centeroids", 1);
    pubObstacles = nh.advertise<obstacle_msgs::Obstacle>("/obstacle_detection/obstacle", 1);

    pnh.param<double>("box_height", boxHeight, 2);
    pnh.param<double>("box_width", boxWidth, 2);

    pnh.param<int>("min_cluster_size", minClusterSize, 2);
    pnh.param<int>("max_cluster_size", maxClusterSize, 400);
    pnh.param<double>("cluster_tolerance", clusterTolerance, 0.1);

    pnh.param<int>("hue_max", hueMax, 80);
    pnh.param<int>("hue_min", hueMin, 60);
    pnh.param<int>("sat_max", satMax, 255);
    pnh.param<int>("sat_min", satMin, 40);
    pnh.param<int>("val_max", valMax, 255);
    pnh.param<int>("val_min", valMin, 40);

    pnh.param<bool>("show_source", showSource, false);
    pnh.param<bool>("show_binary", showBinary, false);

    pnh.param<double>("focal_length_x", f_x, 778.66311);
    pnh.param<double>("focal_length_y", f_y, 776.74570);
    pnh.param<double>("focal_center_x", c_x, 443.39763);
    pnh.param<double>("focal_center_y", c_y, 267.92429);

    pnh.param<double>("dist_coef_k_1", k_1, 0.08284508782990611);
    pnh.param<double>("dist_coef_k_2", k_2, 0.08284508782990611);
    pnh.param<double>("dist_coef_p_1", p_1, 0.08284508782990611);
    pnh.param<double>("dist_coef_p_1", p_2, 0.08284508782990611);
    pnh.param<double>("dist_coef_k_3", k_3, 0.08284508782990611);

    timePointElapsed = 0;
    timePointPrev = ros::Time::now();

    RT = (Mat_<double>(3,4) <<
          0, -1, 0, 0,
          0, 0, -1, -0.05,
          1, 0, 0, -0.16);

    CM = (Mat_<double>(3, 3) <<
          f_x, 0.0, c_x,
          0.0, f_y, c_y,
          0.0, 0.0, 1.0);

    distortion = (Mat_<double>(1, 5) << k_1, k_2, p_1, p_2, k_3);

    //    classify.layout.dim.clear();
    //    classify.data.clear();
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
    if (pclCloudFiltered.size() != 0)
    {
        clustersNum = clustering(pclCloudFiltered.makeShared(), pclClusterVector);
    }

    // merge pcl cluster vector to single cloud
    pclClusters.clear();
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
    pclCenteroids.clear();
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

    timePointElapsed = ros::Time::now().toSec() - timePointPrev.toSec();
    timePointPrev = ros::Time::now();

    ROS_INFO("ELAPSED : %lf [ms]", timePointElapsed * 1000);
    ROS_INFO("seq : %d", scan.header.seq);
    ROS_INFO("cluster size : %d\n", static_cast<int>(clustersNum));

    if(!src.empty())
    {
        Mat hsv;
        Mat srcBin = Mat::zeros(src.size(), src.type());
        cvtColor(src, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(hueMin, satMin, valMin), Scalar(hueMax, satMax, valMax),srcBin);
        erodeAndDilate(srcBin, MORPH_RECT, Size(10,10), 3);
        if(showBinary) imshow("bin", srcBin);

        Mat labels;
        Mat stats;
        Mat tmpCenteroids;
        connectedComponentsWithStats(srcBin, labels, stats, tmpCenteroids, 8);

        std::vector<Rect> rois;
        std::vector<uint32_t> classify;
        rois.reserve(static_cast<size_t>(clustersNum));
        classify.reserve(static_cast<size_t>(clustersNum));
        int* statsPtr = nullptr;
        for (int r = 1; r < stats.rows; ++r)
        {
            statsPtr = stats.ptr<int>(r);
            rois.emplace_back(statsPtr[LEFT_TOP_X], statsPtr[LEFT_TOP_Y], statsPtr[WIDTH], statsPtr[HEIGHT]);
        }

        for(size_t i=0; i < pclCenteroids.size(); i++)
        {
            Point iPt;
            const pcl::PointXYZI& lPt = pclCenteroids[i];
            uint32_t cls = 0;
            imageProjection(lPt, iPt);
            for (size_t r = 0; r < rois.size(); r++)
            {
                const Rect& roi = rois[r];
                if((lPt.x > 0) && roi.contains(iPt))
                {
                    cls = 1; // '1' means the obstacle is a car, otherwise unknown
                }
            }
            classify.push_back(cls);
        }

        // publish to decision maker node
        obstacles.centeroids.clear();
        obstacles.centeroids.reserve(static_cast<size_t>(clustersNum));
        obstacles.labels.clear();
        obstacles.labels.reserve(static_cast<size_t>(clustersNum));
        for(size_t i = 0; i < static_cast<size_t>(clustersNum); i++)
        {
            const pcl::PointXYZI& lPt = pclCenteroids[i];
            geometry_msgs::Point pt;
            pt.x = static_cast<double>(lPt.x);
            pt.y = static_cast<double>(lPt.y);
            pt.z = 0.0;
            obstacles.centeroids.push_back(pt);
            std_msgs::UInt32 num;
            num.data = classify[i];
            obstacles.labels.push_back(num);
        }
        pubObstacles.publish(obstacles);

        // projection to image coordinate
        std::vector<Vec3b> colors(static_cast<size_t>(clustersNum));
        for(size_t i = 0; i < static_cast<size_t>(clustersNum); i++)
        {
            unsigned char B = static_cast<unsigned char>(255 / static_cast<unsigned long>(clustersNum) * (i + 1));
            unsigned char G = static_cast<unsigned char>(255 / static_cast<unsigned long>(clustersNum) * (i + 2));
            unsigned char R = static_cast<unsigned char>(255 / static_cast<unsigned long>(clustersNum) * (i + 3));
            colors[i] = Vec3b(B, G, R);
        }

        Mat prj = src.clone();
        for(size_t i = 0; i < rois.size(); i++)
        {
            rectangle(prj, rois[i], Scalar(0,255,0));
        }
        for(size_t i=0; i < pclCenteroids.size(); i++)
        {
            Point iPt;
            const pcl::PointXYZI& lPt = pclCenteroids[i];
            if(lPt.x > 0)
            {
                imageProjection(lPt, iPt);
                circle(prj, iPt, 5, Scalar(0,0,255), -1);
                if(classify[static_cast<size_t>(lPt.intensity)] == 0) // obstacle
                {
                    putText(prj, "OBSTACLE", iPt, 2, 1.2, Scalar(255,255,255));
                }
                else if(classify[static_cast<size_t>(lPt.intensity)] == 1) // car
                {
                    putText(prj, "CAR", iPt, 2, 1.2, Scalar(255,255,255));
                }
            }
        }

        for(size_t i = 0; i < pclClusters.size(); i++)
        {
            Point iPt;
            pcl::PointXYZI & lPt = pclClusters[i];
            if(lPt.x < 0) continue;

            imageProjection(lPt, iPt);
            double dist = std::sqrt(std::pow(lPt.x,2) + std::pow(lPt.y,1));
            int radius = static_cast<int>(4 / dist);
            if(radius <= 0) radius = 1;

            if(Rect(0,0,prj.cols, prj.rows).contains(iPt)) circle(prj, iPt, radius, colors[static_cast<size_t>(lPt.intensity)], -1);
        }

        if(showSource) imshow("obstacle image", prj);
        waitKey(1);
    }
}

void ObstacleDetection::imgCallback(const sensor_msgs::CompressedImage::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat distorted = cvPtr->image;
    undistort(distorted, src, CM, distortion);
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

void ObstacleDetection::erodeAndDilate(Mat &input, int shape, Size kSize, int repeat)
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

void ObstacleDetection::imageProjection(const pcl::PointXYZI &lPt, Point &iPt)
{
    Mat Xw = (Mat_<double>(4, 1) << lPt.x, lPt.y, lPt.z, 1.0);
    Mat Xc = RT * Xw;
    double s = Xc.ptr<double>(0)[Z];
    Mat Xi = 1 / s * (CM * Xc);

    double* data = Xi.ptr<double>(0);
    iPt.x = data[X];
    iPt.y = data[Y];
}














