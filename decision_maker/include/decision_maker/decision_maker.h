#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include "decision_maker/controller.h"

class DecisionMaker
{
public:
    DecisionMaker(ros::NodeHandle & nh, ros::NodeHandle & pnh);
private:

    // ROS Callbacks
    void timerCallback(const ros::TimerEvent& event);
    void targetSteerCallback(const std_msgs::Float64::ConstPtr& msg);
    void obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void onLaneCallback(const std_msgs::String::ConstPtr& msg);

    // ROS Service
//    ros::Publisher pubCmdVel;
    ros::Subscriber subTargetSteer;
    ros::Subscriber subObstacle;
    ros::Subscriber subOnLane;
    ros::Timer timer;

    // ROS Param
    double targetVel;
    double curveThresholdSteer;
    double curveThresholdTime;

    // Enums
    enum OnLaneType
    {
        ON_LANE_LEFT = 1,
        ON_LANE_RIGHT = -1
    };
    enum GoLaneType
    {
        GO_LANE_LEFT = 1,
        GO_LANE_RIGHT = -1
    };
    enum LaneType
    {
        LANE_CURVE = -1,
        LANE_STRAIGHT = 1
    };

    // Variables
    Controller ctrl;

    double targetSteer;
    double velocity;
    double steer;

    OnLaneType onLane;
    OnLaneType currentLane;
    GoLaneType goLane;
    LaneType lane;
    ros::Time timePointLaneCheck;

    pcl::PointCloud<pcl::PointXYZI> centeroids;
};

#endif // DECISION_MAKER_H
