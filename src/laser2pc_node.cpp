//
// Created by bruce on 2021/2/10.
//


#include <chrono>
#include "tf/transform_listener.h"
#include "laser_geometry/laser_geometry.h"

using namespace std;

class Converter
{
public:
    bool testMode = false;
    bool highFidelity = true;
    string targetFrame = "base_link";

    Converter() : _nh("~")
    {
        _scanSub = _nh.subscribe("/scan", 1, &Converter::scanCallback, this);
        _pcPub = _nh.advertise<sensor_msgs::PointCloud2>("pc", 1);

        _pcTestHighPub = _nh.advertise<sensor_msgs::PointCloud2>("pc_high", 1);
        _pcTestLowPub = _nh.advertise<sensor_msgs::PointCloud2>("pc_low", 1);
    }

private:
    ros::NodeHandle _nh;
    laser_geometry::LaserProjection _projector;
    tf::TransformListener _tfListener;

    ros::Subscriber _scanSub;
    ros::Publisher _pcPub;
    ros::Publisher _pcTestHighPub;
    ros::Publisher _pcTestLowPub;

    void test(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        sensor_msgs::PointCloud2 pcLow;
        auto time1 = chrono::steady_clock::now();
        _projector.projectLaser(*msg, pcLow);
        auto time2 = chrono::steady_clock::now();
        auto duration1 = chrono::duration_cast<chrono::microseconds>(time2 - time1);
        _pcTestLowPub.publish(pcLow);

        sensor_msgs::PointCloud2 pcHigh;
        auto time3 = chrono::steady_clock::now();
        _projector.transformLaserScanToPointCloud("base_link", *msg, pcHigh, _tfListener);
        auto time4 = chrono::steady_clock::now();
        auto duration2 = chrono::duration_cast<chrono::microseconds>(time4 - time3);
        _pcTestHighPub.publish(pcHigh);

        ROS_INFO("low: %ld, high: %ld", duration1.count(), duration2.count());
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        if (testMode) test(msg);
        else
        {
            sensor_msgs::PointCloud2 pc;
            if (highFidelity)
            {
                _projector.transformLaserScanToPointCloud(targetFrame, *msg, pc, _tfListener);
                _pcPub.publish(pc);
            }
            else
            {
                _projector.projectLaser(*msg, pc);
                _pcPub.publish(pc);
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser2pc_node");
    ros::NodeHandle nh("~");

    bool testMode, highFidelity;
    string targetFrame;
    nh.param<bool>("test_mode", testMode, false);
    nh.param<bool>("high_fidelity", highFidelity, true);
    nh.param<string>("target_frame", targetFrame, "base_link");

    Converter converter;
    converter.testMode = testMode;
    converter.highFidelity = highFidelity;
    converter.targetFrame = targetFrame;

    ros::spin();
    return 0;
}