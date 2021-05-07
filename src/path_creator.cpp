//
// Created by bruce on 2021/3/21.
//

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

using namespace std;

class Point
{
public:
    double x;
    double y;

    Point(double x_, double y_)
    {
        set(x_, y_);
    }

    void set(double x_, double y_)
    {
        x = x_;
        y = y_;
    }

    double disTo(double x_, double y_)
    {
        return sqrt(pow(x - x_, 2) + pow(y - y_, 2));
    }

    double disTo(Point &point)
    {
        return disTo(point.x, point.y);
    }

    friend ostream &operator<<(ostream &output, Point &point)
    {
        output << "(" << point.x << "," << point.y << ")";
        return output;
    }
};

class Creator
{
public:
    Creator() : _nh("~")
    {
        _pointSub = _nh.subscribe("point", 1, &Creator::pointCallback, this);
        _goalSub = _nh.subscribe("goal", 1, &Creator::goalCallback, this);
        _pathShowPub = _nh.advertise<nav_msgs::Path>("path_show", 1);
        _pathPub = _nh.advertise<nav_msgs::Path>("path", 1);
    }

private:
    double _resolution = 0.025;
    vector<geometry_msgs::PoseStamped> _keyPoses;
    ros::NodeHandle _nh;

    ros::Subscriber _pointSub;
    ros::Subscriber _goalSub;
    ros::Publisher _pathPub;
    ros::Publisher _pathShowPub;

    geometry_msgs::PoseStamped genPose(double x, double y)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        return pose;
    }

    void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        ROS_INFO("Receive point");
        _keyPoses.emplace_back(genPose(msg->point.x, msg->point.y));

        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        path.poses = _keyPoses;

        _pathShowPub.publish(path);
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // Extract points from keyPoints vector
        // Create a path according to the resolution

        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();

        for (auto &pose : _keyPoses)
        {
            if (path.poses.empty())
                path.poses.emplace_back(pose);
            else
            {
                auto x1 = path.poses.back().pose.position.x;
                auto y1 = path.poses.back().pose.position.y;
                auto x2 = pose.pose.position.x;
                auto y2 = pose.pose.position.y;

                auto dis = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
                auto n = size_t(dis / _resolution);

                auto dx = (x2 - x1) / double(n);
                auto dy = (y2 - y1) / double(n);

                for (size_t i = 0; i < n + 1; i++)
                    path.poses.emplace_back(genPose(x1 + dx * double(i), y1 + dy * double(i)));
            }
        }

        _pathPub.publish(path);
        _keyPoses.clear();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_creator");
    Creator creator;

    ros::spin();
    return 0;
}
