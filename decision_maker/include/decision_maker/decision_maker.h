#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#define _USE_MATH_DEFINES
#include <math.h>

class DecisionMaker
{
public:
    DecisionMaker(ros::NodeHandle & nh, ros::NodeHandle & pnh);
private:

    // ROS Service
    ros::Publisher pubCmdVel;
    ros::Subscriber subTargetSteer;
    ros::Timer timer;

    // ROS Callbacks
    void timerCallback(const ros::TimerEvent& event);
    void targetSteerCallback(const std_msgs::Float64::ConstPtr& msg);

    // variables
    double targetVel;
    double targetSteer;
    double velocity;
};

#endif // DECISION_MAKER_H
