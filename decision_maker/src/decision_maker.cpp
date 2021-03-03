#include "decision_maker/decision_maker.h"
#include <geometry_msgs/Twist.h>

DecisionMaker::DecisionMaker(ros::NodeHandle & nh, ros::NodeHandle & pnh)
    :ctrl(nh, pnh)
{
    subTargetSteer = nh.subscribe("/lane_detection/target_steer", 1, &DecisionMaker::targetSteerCallback, this);
    subOnLane = nh.subscribe("/lane_detection/on_lane", 1, &DecisionMaker::onLaneCallback, this);
    subObstacle = nh.subscribe("/obstacle_detection/point_cloud_cluster_centeroids", 1, &DecisionMaker::obstacleCallback, this);

    timer = nh.createTimer(ros::Duration(1.0 / 20.0), &DecisionMaker::timerCallback, this);
    pnh.param<double>("target_velocity", targetVel, 0.8);
    pnh.param<double>("curve_threshold_steer", curveThresholdSteer, 0.15);
    pnh.param<double>("curve_threshold_time", curveThresholdTime, 0.5);

    goLane = GO_LANE_RIGHT;

    targetSteer = 0;
    timePointLaneCheck = ros::Time::now();
}

void DecisionMaker::timerCallback(const ros::TimerEvent &)
{
    geometry_msgs::Twist target;
    velocity = targetVel;
    steer = targetSteer;

    // check lane type
    ros::Time timePointCurrent = ros::Time::now();
    std::string laneTypeMsg = "UNKNOWN";
    if(std::abs(targetSteer) > curveThresholdSteer)
    {
        timePointLaneCheck = ros::Time::now();
        lane = LANE_CURVE;
        laneTypeMsg = "CURVE";
    }
    if(timePointCurrent.toSec() - timePointLaneCheck.toSec() > curveThresholdTime)
    {
        lane = LANE_STRAIGHT;
        laneTypeMsg = "STRAIGHT";
    }
    ROS_INFO("LANE : %s", laneTypeMsg.c_str());


    // stop when obstacle exist
    bool obstacle = false;
    for(size_t i = 0; i < centeroids.size(); i++)
    {
        const pcl::PointXYZI& pt = centeroids[i];
        if(
                (0.0f < pt.x && pt.x < 0.5f) &&
                (-0.25f < pt.y && pt.y < 0.25f)
                )
        {
            obstacle = true;
        }
    }
    if(obstacle)
    {
        velocity = 0;
        ROS_WARN("OBSTACLE AHEAD");
    }
    else
    {
        ROS_INFO("OBSTACLE FREE");
    }

    if(!ctrl.isMenual())
    {
        ctrl.setTargetVelocity(velocity);
        ctrl.setTargetSteer(steer);
    }
    ctrl.publishControlInput();
}

void DecisionMaker::targetSteerCallback(const std_msgs::Float64::ConstPtr &msg)
{
    const std_msgs::Float64& steer = *msg;
    targetSteer = steer.data;
}

void DecisionMaker::obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    const sensor_msgs::PointCloud2& pcd = *msg;
    pcl::fromROSMsg(pcd, centeroids);
}

void DecisionMaker::onLaneCallback(const std_msgs::String::ConstPtr &msg)
{
    const std_msgs::String& lane = *msg;
    if(lane.data == "ON_LANE_LEFT")
    {
        onLane = ON_LANE_LEFT;
    }
    else if(lane.data == "ON_LANE_RIGHT")
    {
        onLane = ON_LANE_RIGHT;
    }
}
