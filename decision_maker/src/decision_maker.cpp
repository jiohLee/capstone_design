#include "decision_maker/decision_maker.h"
#include <geometry_msgs/Twist.h>

DecisionMaker::DecisionMaker(ros::NodeHandle & nh, ros::NodeHandle & pnh)
{
    pubCmdVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    // subTrackArray = nh.subscribe("/datmo/box_kf", 1, &DecisionMaker::trackArrayCallback, this);

    timer = nh.createTimer(ros::Duration(1.0 / 20.0), &DecisionMaker::timerCallback, this);
    pnh.param<double>("target_velocity", targetVel, 0.8);

    targetSteer = 0;
}

void DecisionMaker::timerCallback(const ros::TimerEvent &)
{
    geometry_msgs::Twist target;
    velocity = targetVel;
    target.linear.x = velocity;
    target.angular.z = targetSteer;
    pubCmdVel.publish(target);
}

void DecisionMaker::targetSteerCallback(const std_msgs::Float64::ConstPtr &msg)
{
    const std_msgs::Float64& steer = *msg;
    targetSteer = steer.data;
}
