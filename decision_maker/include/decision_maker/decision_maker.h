#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Joy.h>
// #include <datmo/TrackArray.h>
// #include <datmo/Track.h>

#include <deque>
#define _USE_MATH_DEFINES
#include <math.h>

class DecisionMaker
{
public:
    DecisionMaker(ros::NodeHandle & nh, ros::NodeHandle & pnh);
private:

    // ROS Service
    ros::Publisher pubCmdVel;
    ros::Subscriber subPoint;
    ros::Subscriber subJoy;
    ros::Subscriber subTrackArray;
    ros::Timer timer;

    void waypointCallback(const geometry_msgs::Point::ConstPtr & msg);
    void timerCallback(const ros::TimerEvent& event);
    void joyCallback(const sensor_msgs::Joy::ConstPtr & msg);
    // void trackArrayCallback(const datmo::TrackArray::ConstPtr & msg);

    std::deque<double> error;
    const int error_cap = 10;

    // ROS parameters
    double p_gain;
    double i_gain;
    double d_gain;
    double steer_offset;

    double t_a;
    double t_b;
    double t_c;

    // variables
    double targetVel;
    double targetSteer;
    double velocity;

    bool bLaneChange = false;
    bool bOnLaneLeft = false;
    bool bOnLaneRight = true;
    bool bObstacle = false;
    ros::Time laneChangeDur;
    bool run;

    // datmo::Track obs;

    //
    enum TurnType
    {
      LANE_LEFT = 1,
      LANE_RIGHT = -1
    };

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

    // Functions
    double getTrajectory(double t, int TurnType);
};

#endif // DECISION_MAKER_H
