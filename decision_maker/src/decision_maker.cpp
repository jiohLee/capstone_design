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

    goLane = GO_LANE_RIGHT;

    targetSteer = 0;
}

void DecisionMaker::timerCallback(const ros::TimerEvent &)
{
    geometry_msgs::Twist target;
    velocity = targetVel;
    steer = targetSteer;

    // stop when obstacle exist
    for(size_t i = 0; i < centeroids.size(); i++)
    {
        const pcl::PointXYZI& pt = centeroids[i];
        if(
                (0.0f < pt.x && pt.x < 2.0f) &&
                (-0.25f < pt.y && pt.y < 0.25f)
                )
        {
            velocity = 0;
        }
    }

    if(!ctrl.isMenual())
    {
        ctrl.setTargetVelocity(velocity);
        ctrl.setTargetSteer(steer);
    }
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
