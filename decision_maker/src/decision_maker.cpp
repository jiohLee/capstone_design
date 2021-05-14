#include "decision_maker/decision_maker.h"
#include <geometry_msgs/Twist.h>

DecisionMaker::DecisionMaker(ros::NodeHandle & nh, ros::NodeHandle & pnh)
    :ctrl(nh, pnh)
{
    subTargetSteer = nh.subscribe("/lane_detection/lane_center_point", 1, &DecisionMaker::laneCenterCallback, this);
    subOnLane = nh.subscribe("/lane_detection/on_lane", 1, &DecisionMaker::onLaneCallback, this);
    subObstacle = nh.subscribe("/obstacle_detection/obstacle", 1, &DecisionMaker::obstacleCallback, this);

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
    // init values
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

    if(isObstacleAhead(0, 1.0, onLane))
    {
        if(car)
        {
            ROS_INFO("CAR AHEAD");
            if(carPoint.x < 0.7)
            {
                switchLane();
            }
        }
        else
        {
            ROS_INFO("OBSACLE AHEAD");
        }
    }
    else
    {
        ROS_INFO("OBSTACLE FREE");
        if(onLane == ON_LANE_LEFT)
        {
            switchLane();
        }
    }

    // info
    if(onLane == ON_LANE_LEFT)
    {
        ROS_INFO("ON_LANE_LEFT");
        velocity = 0.8;
    }
    else
    {
        velocity = 0.6;
        ROS_INFO("ON_LANE_RIGHT");
    }
    if(goLane == GO_LANE_LEFT)
    {
        ROS_INFO("GO_LANE_LEFT");
    }
    else
    {
        ROS_INFO("GO_LANE_RIGHT");
    }

    // follow order
    if(goLane == GO_LANE_RIGHT)
    {
        if(onLane == ON_LANE_LEFT)
        {
            steer = GO_LANE_RIGHT * 0.3;
            velocity = 0.4;
        }
    }
    else if(goLane == GO_LANE_LEFT)
    {
        if(onLane == ON_LANE_RIGHT)
        {
            steer = GO_LANE_LEFT * 0.3;
            velocity = 0.4;
        }
    }

    // estop

    if(isObstacleAhead(0, 0.5, onLane))
    {
        velocity = 0;
    }

    if(!ctrl.isMenual())
    {
        ctrl.setTargetVelocity(velocity);
        ctrl.setTargetSteer(steer);
    }
    ctrl.publishControlInput();
}

void DecisionMaker::laneCenterCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    const geometry_msgs::Point& pt = *msg;
    targetSteer = std::atan2(pt.y, pt.x);
}

void DecisionMaker::obstacleCallback(const obstacle_msgs::Obstacle::ConstPtr &msg)
{
    obstacles = *msg;
    geometry_msgs::Point pt;
    if(isCarDetected(pt))
    {
        car = true;
        carPoint = pt;
    }
    else
    {
        car = false;
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
        ROS_INFO("READY TO SWITCH");
        if(!isObstacleAhead(-1.0, 1.0, static_cast<OnLaneType>(onLane * -1)))
        {
            ROS_INFO("SWITCHING");
            if(onLane == ON_LANE_LEFT)
            {
                goLane = GO_LANE_RIGHT;
            }
            else if(onLane == ON_LANE_RIGHT)
            {
                goLane = GO_LANE_LEFT;
            }
        }
        else
        {
            ROS_INFO("CANNOT SWITCH");
        }
    }
}

bool DecisionMaker::isCarDetected(geometry_msgs::Point &pt)
{
    for(size_t i = 0; i < obstacles.labels.size(); i++)
    {
        if(obstacles.labels[i].data == 1)
        {
            pt = obstacles.centeroids[i];
            return true;
        }
    }
    return false;
}

bool DecisionMaker::isObstacleAhead(double lowerRange, double upperRange, DecisionMaker::OnLaneType olt)
{
    double yRangeLeft = -0.25;
    double yRangeRight = 0.25;

    if(olt == ON_LANE_LEFT)
    {
        if(onLane == ON_LANE_RIGHT)
        {
            yRangeLeft += 0.4;
            yRangeRight += 0.4;
        }
    }
    else if(olt == ON_LANE_RIGHT)
    {
        if(onLane == ON_LANE_LEFT)
        {
            yRangeLeft -= 0.4;
            yRangeRight -= 0.4;
        }
    }

    for(size_t i = 0; i < obstacles.centeroids.size(); i++)
    {
        const geometry_msgs::Point& pt = obstacles.centeroids[i];
        if(
                (lowerRange < pt.x && pt.x < upperRange) &&
                (yRangeLeft < pt.y && pt.y < yRangeRight)
                )
        {
            return true;
        }
    }
    return false;
}
