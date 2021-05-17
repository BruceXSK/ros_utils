//
// Created by bruce on 2021/5/17.
//

#include "ros/ros.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guard_demo");
    ros::NodeHandle nh("~");

    auto loopRate = ros::Rate(1);
    while (ros::ok())
    {
        ROS_INFO("Hello");
        loopRate.sleep();
    }

    return 0;
}