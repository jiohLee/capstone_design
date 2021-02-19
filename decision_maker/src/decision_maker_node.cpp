#include <ros/ros.h>
#include "decision_maker/decision_maker.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "decision_maker_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    DecisionMaker dm(nh, pnh);
    ros::spin();
    return 0;
}
