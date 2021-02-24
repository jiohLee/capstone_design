#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>
#include <deque>

class Controller
{
public:
    Controller(ros::NodeHandle nh, ros::NodeHandle pnh);

    void setTargetVelocity(const double target);
    void setTargetSteer(const double target);
private:

    // ROS Callbacks
    void joyCallback(const sensor_msgs::Joy::ConstPtr & msg);
    void ctrlInputCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void ctrlTimerCallback(const ros::TimerEvent& event);

    // ROS Services
    ros::Subscriber subJoy;
    ros::Subscriber subTwist;
    ros::Publisher pubAck;
    ros::Timer ctrlTimer;

    // ROS param
    double accel;
    double steerOffset;
    double pGain;
    double iGain;
    double dGain;
    bool menualStart;

    // Functions
    void setVelocity(const double timeElapsedSec, const double accelMperSec);
    void setSteer(const double pGain, const double iGain, const double dGain);

    // Joy Message Index
    enum AXES_IDX
    {
        AXES_LEFT_RL = 0,   // left stick left/right index
        AXES_LEFT_UD = 1,   // left stick up/down index
        AXES_RIGHT_RL = 2,
        AXES_RIGHT_UD = 3,
        AXES_CROSS_RL = 4,
        AXES_CROSS_UD = 5
    };

    enum BUTTON_IDX
    {
        BT_X = 0,   // X Button
        BT_A = 1,   // A Button
        BT_B = 2,   // B Button
        BT_Y = 3,   // Y Button
        BT_LB = 4,   // LB Button
        BT_RB = 5,   // RB Button
        BT_LT = 6,   // LT Button
        BT_RT = 7,   // RT Button
    };

    // Variables
    const double steeringRatio = 0.4;

    ros::Time prevTime;

    double targetVelocity;
    double targetSteer;
    double velocity;    // m/s
    double steer;       // radian

    std::deque<double> steeringError;
    const size_t queueSize = 10;

    bool menual;
};

#endif // CONTROLLER_H
