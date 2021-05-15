#include "decision_maker/controller.h"

Controller::Controller(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    pubAck = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd_input", 1);
    subJoy = nh.subscribe("/joy", 1, &Controller::joyCallback, this);

    pnh.param<double>("accel", accel, 0.2);
    pnh.param<double>("steer_offset", steerOffset, 0.05);

    pnh.param<double>("p_gain", pGain, 0.6);
    pnh.param<double>("i_gain", iGain, 0.005);
    pnh.param<double>("d_gain", dGain, 2);

    pnh.param<bool>("menual_start", menualStart, true);

    targetVelocity = 0.0;
    targetSteer = 0.0;
    velocity = 0.0;
    steer = 0.0;

    menual = menualStart;
    stop = true;

    steeringError.clear();
    prevTime = ros::Time::now();
}

void Controller::setTargetVelocity(const double target)
{
    if(target == 0.0)
    {
        stop = true;
        return;
    }
    stop = false;

    const double velocityMax = 1;
    if (target > velocityMax) targetVelocity = velocityMax;
    else if(target < -velocityMax) targetVelocity = -velocityMax;
    else targetVelocity = target;
}

void Controller::setTargetSteer(const double target)
{
    const double steerMax = 0.4;
    if(target > steerMax) targetSteer = steerMax;
    else if(target < -steerMax) targetSteer = -steerMax;
    else targetSteer = target;

    if(steeringError.size() < queueSize)
    {
        steeringError.push_back(target);
    }
    else
    {
        steeringError.pop_front();
        steeringError.push_back(target);
    }
}

void Controller::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    /*
     * axes array float size 6
     *
     * [0] 1.0 : left - left max / -1.0 : left - right min
     * [1] 1.0 : left - up max / -1.0 : left - down min
     * [2] 1.0 : right - left max / -1.0 : right - right min
     * [3] 1.0 : right - up max / -1.0 : right - down min
     * [4] 1.0 : cross - left max / -1.0 : cross - right min
     * [5] 1.0 : cross - up max / -1.0 : cross - down min
     *
     * buttons array int size 12
     *
     * [0] : x
     * [1] : a
     * [2] : b
     * [3] : y
     * [4] : LB
     * [5] : RB
     * [6] : LT
     * [7] : RT
     * [8~11] : small black buttons in middle of joystick. do not use
    */

    const sensor_msgs::Joy & joy = *msg;

    if(joy.buttons[BT_LB])
    {
        menual = true;
    }
    else if(joy.buttons[BT_RB])
    {
        menual = false;
    }

    if(menual)
    {
        setTargetSteer(static_cast<double>(joy.axes[AXES_RIGHT_RL]) * steeringRatio);
        setTargetVelocity(static_cast<double>(joy.axes[AXES_LEFT_UD]));
    }
}

void Controller::publishControlInput()
{
    ros::Time currTime = ros::Time::now();
    double timeElapsed = currTime.toSec() - prevTime.toSec();

    setVelocity(timeElapsed, accel);
    setSteer(pGain, iGain, dGain);

    if(menual)
    {
        ROS_INFO("CONTROL : %s", std::string("MENUAL").c_str());
    }
    else
    {
        ROS_WARN("CONTROL : %s", std::string("AUTO").c_str());
    }
    ROS_INFO("VELOCITY : %f [m/sec]", velocity);
    ROS_INFO("STEER : %lf [rad]", steer);
    ROS_INFO("DURATION : %f [sec]\n", timeElapsed);
    printf("\n");

    ackermann_msgs::AckermannDriveStamped ackmsg;
    ackmsg.header.stamp = ros::Time::now();
    ackmsg.drive.speed = static_cast<float>(velocity);
    ackmsg.drive.steering_angle = static_cast<float>(steer);
    pubAck.publish(ackmsg);

    prevTime = currTime;
}

void Controller::setVelocity(const double timeElapsedSec, const double accelMperSec)
{
    if(stop) velocity = 0;
    else if(targetVelocity > velocity)
    {
        if ( velocity + (accelMperSec * timeElapsedSec) > targetVelocity)
        {
            velocity = targetVelocity;
        }
        else
        {
            velocity += (accelMperSec * timeElapsedSec);
        }
        if (velocity > 1) velocity = 1;
        if (targetVelocity > 0.2 && velocity < 0.2 ) velocity = 0.2;
    }
    else if (targetVelocity < velocity)
    {
        if ( velocity - (accelMperSec * timeElapsedSec) < targetVelocity)
        {
            velocity = targetVelocity;
        }
        else
        {
            velocity -= (accelMperSec * timeElapsedSec);
        }
        if (velocity < -1) velocity = -1;
        if (targetVelocity < -0.2 && velocity > -0.2 ) velocity = -0.2;
    }
}

void Controller::setSteer(const double pGain, const double iGain, const double dGain)
{
    if(menual)
    {
        steer = targetSteer;
    }
    else // PID Control
    {
        double integral = 0;
        for (size_t i = 0; i < 10; i ++)
        {
            integral += steeringError[i];
        }
        double derivative = steeringError[9] - steeringError[8];
        steer = pGain * steeringError[9] + dGain * derivative + iGain * integral;
        steer -= steerOffset;
    }
}

