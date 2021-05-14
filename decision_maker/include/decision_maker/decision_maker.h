#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include "decision_maker/controller.h"

#include <obstacle_msgs/Obstacle.h>

class DecisionMaker
{
public:
    DecisionMaker(ros::NodeHandle & nh, ros::NodeHandle & pnh);
private:

    // ROS Callbacks
    void timerCallback(const ros::TimerEvent& event);
    void laneCenterCallback(const geometry_msgs::Point::ConstPtr& msg);
    void obstacleCallback(const obstacle_msgs::Obstacle::ConstPtr& msg);
    void onLaneCallback(const std_msgs::String::ConstPtr& msg);

    // ROS Service
    ros::Subscriber subTargetSteer;
    ros::Subscriber subObstacle;
    ros::Subscriber subOnLane;
    ros::Timer timer;

    // ROS Messages
    obstacle_msgs::Obstacle obstacles;

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

    // Functions
    void switchLane();
    bool isCarDetected(geometry_msgs::Point& pt);
    bool isObstacleAhead(double lowerRange, double upperRange, OnLaneType olt);

    // Variables
    Controller ctrl;

    double targetSteer;
    double velocity;
    double steer;

    OnLaneType onLane;
    GoLaneType goLane;
    LaneType lane;
    ros::Time timePointLaneCheck;
    ros::Time timePointStraightCheck;

    bool car;
    geometry_msgs::Point carPoint;

    // experiment
    std::vector<geometry_msgs::Point> lookAheadPoints;
};

#endif // DECISION_MAKER_H
