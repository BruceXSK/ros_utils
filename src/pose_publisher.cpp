//
// Created by bruce on 2021/2/6.
//

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_publisher");
    ros::NodeHandle nh("~");

    ros::Rate loopRate(10);
    tf::TransformListener tfListener;
    tf::StampedTransform transform;

    auto posePub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    while (ros::ok())
    {
        bool tf_ok = true;
        try
        {
            tfListener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        }
        catch (exception &e)
        {
            ROS_WARN("%s", e.what());
            tf_ok = false;
        }

        if (tf_ok)
        {
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header.stamp = ros::Time::now();
            poseStamped.header.frame_id= "map";

            poseStamped.pose.position.x = transform.getOrigin().getX();
            poseStamped.pose.position.y = transform.getOrigin().getY();
            poseStamped.pose.position.z = transform.getOrigin().getZ();

            poseStamped.pose.orientation.x = transform.getRotation().getX();
            poseStamped.pose.orientation.y = transform.getRotation().getY();
            poseStamped.pose.orientation.z = transform.getRotation().getZ();
            poseStamped.pose.orientation.w = transform.getRotation().getW();

            posePub.publish(poseStamped);
        }

        loopRate.sleep();
    }

    return 0;
}
