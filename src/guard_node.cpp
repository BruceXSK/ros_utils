//
// Created by bruce on 2021/5/17.
//

#include "ros/ros.h"
#include "std_msgs/Empty.h"

using namespace std;

bool run = true;

void killCallback(const std_msgs::Empty::ConstPtr &msg)
{
    ROS_INFO("Received the KILL signal");
    run = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guard_node");
    ros::NodeHandle nh("~");

    auto killSub = nh.subscribe("kill", 1, killCallback);

    while (ros::ok() and run)
    {
        ros::spinOnce();
    }

    return 0;
}