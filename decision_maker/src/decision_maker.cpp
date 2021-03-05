#include "decision_maker/decision_maker.h"
#include <geometry_msgs/Twist.h>

DecisionMaker::DecisionMaker(ros::NodeHandle & nh, ros::NodeHandle & pnh)
    :ctrl(nh, pnh)
{
    subTargetSteer = nh.subscribe("/lane_detection/target_steer", 1, &DecisionMaker::targetSteerCallback, this);
    subOnLane = nh.subscribe("/lane_detection/on_lane", 1, &DecisionMaker::onLaneCallback, this);
    subObstacle = nh.subscribe("/obstacle_detection/point_cloud_cluster_centeroids", 1, &DecisionMaker::obstacleCallback, this);
    subClassify = nh.subscribe("/obstacle_detection/point_cloud_cluster_class", 1, &DecisionMaker::classifyCallback, this);

    timer = nh.createTimer(ros::Duration(1.0 / 20.0), &DecisionMaker::timerCallback, this);
    pnh.param<double>("target_velocity", targetVel, 0.8);
    pnh.param<double>("curve_threshold_steer", curveThresholdSteer, 0.15);
    pnh.param<double>("curve_threshold_time", curveThresholdTime, 0.5);

    goLane = GO_LANE_RIGHT; // start on the right lane

    car = false;

    targetSteer = 0;
    timePointLaneCheck = ros::Time::now();
}

void DecisionMaker::timerCallback(const ros::TimerEvent &)
{
    geometry_msgs::Twist target;
    velocity = targetVel;
    steer = targetSteer;

    // check lane type
    std::string laneTypeMsg = "UNKNOWN";
    double laneDur = ros::Time::now().toSec() - timePointLaneCheck.toSec();
    if(std::abs(targetSteer) > curveThresholdSteer)
    {
        timePointLaneCheck = ros::Time::now();
        lane = LANE_CURVE;
        laneTypeMsg = "CURVE";
    }
    if(laneDur > curveThresholdTime)
    {
        lane = LANE_STRAIGHT;
        laneTypeMsg = "STRAIGHT";
    }
    ROS_INFO("LANE : %s", laneTypeMsg.c_str());

    // check car ahead

    if(!car)
    {
        if(isCarDetected(carPoint))
        {
            car = true;
            ROS_INFO("CAR DETECTED");
        }
        else
        {
            ROS_INFO("NO CAR");
        }
    }
    else
    {
        carPointPrev = carPoint;
        if(isCarDetected(carPoint))
        {
            double relativeVelocity = (carPoint.x - carPointPrev.x) / 0.05;
//            std::cout << carPoint << "\n" << carPointPrev << "\n";
            ROS_INFO("CAR DETECTED, RELVEL : %lf", relativeVelocity);
        }
        else
        {
            ROS_INFO("NO CAR");
        }
    }

    // follow order
    if(goLane == GO_LANE_RIGHT)
    {
        if(onLane == ON_LANE_LEFT)
        {
            steer = GO_LANE_RIGHT * 0.4;
        }
    }
    else if(goLane == GO_LANE_LEFT)
    {
        if(onLane == ON_LANE_RIGHT)
        {
            steer = GO_LANE_LEFT * 0.4;
        }
    }
    // slow down when curve appear
    if(lane == LANE_CURVE)
    {
        velocity = 0.4;
    }

    // estop
    bool obstacle = false;
    for(size_t i = 0; i < centeroids.size(); i++)
    {
        const pcl::PointXYZI& pt = centeroids[i];
        if(
                (0.0f < pt.x && pt.x < 0.5f) &&
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

void DecisionMaker::classifyCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
    const std_msgs::UInt8MultiArray & arr = *msg;
    classify.clear();
    for(size_t i = 0; i < arr.data.size(); i++)
    {
        classify.push_back(arr.data[i]);
    }
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

void DecisionMaker::switchLane()
{
    if(lane == LANE_STRAIGHT)
    {
        if(onLane == ON_LANE_LEFT)
        {
            goLane = GO_LANE_RIGHT;
        }
        else if(onLane == ON_LANE_RIGHT)
        {
            goLane = GO_LANE_LEFT;
        }
    }
}

bool DecisionMaker::isCarDetected(pcl::PointXYZI& point)
{
    for(size_t i = 0; i < classify.size(); i++)
    {
         if(classify[i] == 1) // car
         {
            point = centeroids[i];
            std::cout << point << "\n";
            return true;
         }
    }
    return false;
}
