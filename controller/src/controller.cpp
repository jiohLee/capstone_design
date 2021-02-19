#include "controller/controller.h"

Controller::Controller(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    pubAck = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);
    subJoy = nh.subscribe("/joy", 1, &Controller::joyCallback, this);
    subTwist = nh.subscribe("/cmd_vel", 1, &Controller::ctrlInputCallback, this);
    ctrlTimer = nh.createTimer(ros::Duration(1.0 / 20.0), &Controller::ctrlTimerCallback, this);
    pnh.param<double>("accel", accel, 0.2);
    pnh.param<bool>("joy_velocity_steer_control", joyVelocitySteerControl, true);
    pnh.param<bool>("run", run, true);

    velocity = 0.0;
    targetVelocity = 0;
    steer = 0.0;
    estop = false;
    brake = false;
}

void Controller::setTargetVelocity(const double target)
{
    brake = false;
    const double velocityMax = 1;
    if (target > velocityMax) targetVelocity = velocityMax;
    else if(target < -velocityMax) targetVelocity = -velocityMax;
    else targetVelocity = target;
}

void Controller::setSteer(const double target)
{
    const double steerMax = 0.4;
    if(target > steerMax) steer = steerMax;
    else if(target < -steerMax) steer = -steerMax;
    else steer = target;
}

void Controller::putBrake()
{
    brake = true;
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

    if(joyVelocitySteerControl)
    {
        setSteer(static_cast<double>(joy.axes[AXES_RIGHT_RL]) * steeringRatio);
        setTargetVelocity(static_cast<double>(joy.axes[AXES_LEFT_UD]));
    }

    if (joy.buttons[BT_RB] == 1) // press RB button -> EMERGENCY STOP
    {
        estop = true;
    }
    else
    {
        estop = false;
    }

    if (joy.buttons[BT_Y] == 1)
    {
        run = true;
    }
    else if (joy.buttons[BT_B] == 1)
    {
        run = false;
    }
}

void Controller::ctrlInputCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    const geometry_msgs::Twist & tw = *msg;
    setSteer(tw.angular.z);
    setTargetVelocity(tw.linear.x);
}

void Controller::ctrlTimerCallback(const ros::TimerEvent &)
{
    ros::Time currTime = ros::Time::now();
    double durTime = currTime.toSec() - prevTime.toSec();
    std::cout << "DURATION : " << durTime << "\n";

    // if press Y -> start
    if (run)
    {
        ROS_INFO("RACECAR RUN");
        // follow target
        if(targetVelocity > velocity)
        {
            if ( velocity + (accel * durTime) > targetVelocity)
            {
                velocity = targetVelocity;
            }
            else
            {
                velocity += (accel * durTime);
            }
            if (velocity > 1) velocity = 1;
            if (targetVelocity > 0.2 && velocity < 0.2 ) velocity = 0.2;
        }
        else if (targetVelocity < velocity)
        {
            if ( velocity - (accel * durTime) < targetVelocity)
            {
                velocity = targetVelocity;
            }
            else
            {
                velocity -= (accel * durTime);
            }
            if (velocity < -1) velocity = -1;
            if (targetVelocity < -0.2 && velocity > -0.2 ) velocity = -0.2;
        }
    }
    else
    {
        brake = true;
        ROS_INFO("RACECAR STOP");
    }

    if(estop | brake)
    {
        targetVelocity = 0;
        velocity = 0;
        steer = 0;
        ROS_WARN("RACECAR BRAKE");
    }

    std::cout
            << "CURRENT VEL : " << velocity << "\n"
            << "CURRENT STEER : " << steer << "\n\n";

    ackermann_msgs::AckermannDriveStamped ackmsg;
    ackmsg.header.stamp = ros::Time::now();
    ackmsg.drive.speed = static_cast<float>(velocity);
    ackmsg.drive.steering_angle = static_cast<float>(steer);
    pubAck.publish(ackmsg);

    prevTime = currTime;
}
