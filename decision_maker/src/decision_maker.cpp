#include "decision_maker/decision_maker.h"
#include <geometry_msgs/Twist.h>

DecisionMaker::DecisionMaker(ros::NodeHandle & nh, ros::NodeHandle & pnh)
{
    subPoint = nh.subscribe("/waypoint", 1, &DecisionMaker::waypointCallback, this);
    pubCmdVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    subJoy = nh.subscribe("/joy", 1, &DecisionMaker::joyCallback, this);
    // subTrackArray = nh.subscribe("/datmo/box_kf", 1, &DecisionMaker::trackArrayCallback, this);

    timer = nh.createTimer(ros::Duration(1.0 / 20.0), &DecisionMaker::timerCallback, this);
    pnh.param<double>("target_velocity", targetVel, 0);
    pnh.param<double>("p_gain", p_gain, 0.6);
    pnh.param<double>("i_gain", i_gain, 0.6);
    pnh.param<double>("d_gain", d_gain, 0.6);
    pnh.param<double>("steer_offset", steer_offset, 0.05);

    pnh.param<double>("trajectory_param_a", t_a, 1.0);
    pnh.param<double>("trajectory_param_b", t_b, 1.0);
    pnh.param<double>("trajectory_param_c", t_c, 1.5);
    pnh.param<bool>("run", run, false);

    std::string startLane;
    pnh.param<std::string>("start_lane", startLane, "left");

    if(startLane == "left")
    {
        bOnLaneLeft = true;
        bOnLaneRight = false;
    }
    else if(startLane == "right")
    {
        bOnLaneLeft = false;
        bOnLaneRight = true;
    }

    targetSteer = 0;
    laneChangeDur = ros::Time::now();
}

void DecisionMaker::waypointCallback(const geometry_msgs::Point::ConstPtr & msg)
{
    const geometry_msgs::Point & data = *msg;

    if(static_cast<int>(error.size()) < error_cap)
    {
        error.push_back(atan2(data.y, data.x));
    }
    else
    {
        error.pop_front();
        error.push_back(atan2(data.y, data.x));
    }
}

void DecisionMaker::timerCallback(const ros::TimerEvent &)
{
    geometry_msgs::Twist target;

    double integral = 0;
    for (int i = 0; i < 10; i ++)
    {
        integral += error[i];
    }

    double derivative = error[9] - error[8];

    targetSteer = p_gain * error[9] + d_gain * derivative + i_gain * integral;
    velocity = targetVel;

    // get lane change time
    if(!run) laneChangeDur = ros::Time::now();

    if( /*ros::Time::now().toSec() - laneChangeDur.toSec() > 2*/ bObstacle && bOnLaneRight && !bLaneChange && targetSteer < 0.03 && run)
    {
        bLaneChange = true;
        laneChangeDur = ros::Time::now();
    }
    else if ( !bObstacle && ros::Time::now().toSec() - laneChangeDur.toSec() > 2 && bOnLaneLeft && !bLaneChange && targetSteer < 0.03 && run)
    {
        bLaneChange = true;
        laneChangeDur = ros::Time::now();
    }
    else if( bObstacle )
    {
        velocity = 0.6;
    }

    if(bLaneChange)
    {
        velocity = 0.6;
        double t = ros::Time::now().toSec() - laneChangeDur.toSec();
        if( t > 1)
        {
            if(bOnLaneRight)
            {
                targetSteer = getTrajectory(t - 1, LANE_LEFT);
            }
            else if(bOnLaneLeft)
            {
                targetSteer = getTrajectory(t - 1, LANE_RIGHT);
            }

            if(t - 1 > (t_b + t_c * t_a) * targetVel)
            {
                laneChangeDur = ros::Time::now();
                bLaneChange = false;
                if(bOnLaneLeft)
                {
                    bOnLaneRight = true;
                    bOnLaneLeft = false;
                }
                else if(bOnLaneRight)
                {
                    bOnLaneRight = false;
                    bOnLaneLeft = true;
                }
            }
        }
    }

    if(!bLaneChange) targetSteer = targetSteer - steer_offset;
    target.linear.x = velocity;
    target.angular.z = targetSteer;
    pubCmdVel.publish(target);
}

void DecisionMaker::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
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

    if (joy.buttons[BT_Y] == 1)
    {
        run = true;
    }
    else if (joy.buttons[BT_B] == 1)
    {
        run = false;
    }
}

// void DecisionMaker::trackArrayCallback(const datmo::TrackArray::ConstPtr &msg)
// {
//     const datmo::TrackArray & tarr = *msg;
//     int length = tarr.tracks.size();
//     for (int i = 0; i < length; i++)
//     {
//         const datmo::Track t_ = tarr.tracks[i];
//         double id = t_.id;
//         double x_ = t_.odom.pose.pose.position.x;
//         double y_ = t_.odom.pose.pose.position.y;

//         if(
//             -0.05 <= y_ && y_ <= 0.05 &&
//             x_ < 0
//                 )
//         {
//             bObstacle = true;
//             std::cout
//                     << "x : " << x_
//                     << " y : " << y_ << "\n";
//             obs = t_;
//             break;
//         }
//         else
//         {
//             bObstacle = false;
//         }

//     }

//     if(bObstacle) ROS_WARN("Obstacle!");
//     else if(!bObstacle) ROS_INFO("NO Obstacle!");
//     std::cout << "\n" << std::endl;
// }

double DecisionMaker::getTrajectory(double t, int TurnType)
{
   double steer = 0.3;
   if(TurnType == LANE_LEFT)
   {
       if(t < t_a * velocity )
       {
           steer = steer * TurnType;
       }
       else if((t_a + t_b) * velocity <= t && t <= (t_a + t_b + t_c) * velocity)
       {
          steer = steer * (TurnType * -1);
       }
       else
       {
           steer = 0;
       }
   }
   else if(TurnType = LANE_RIGHT)
   {
       if(t < t_c * velocity )
       {
           steer = steer * TurnType;
       }
       else if((t_c + t_b) * velocity <= t && t <= (t_a + t_b + t_c) * velocity)
       {
          steer = steer * (TurnType * -1);
       }
       else
       {
           steer = 0;
       }
   }

   return steer;
}
