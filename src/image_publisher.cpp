//
// Created by bruce on 2021/4/23.
//

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh("~");

    const auto& nodeName = ros::this_node::getName();

    string port;
    if (not nh.getParam("port", port))
    {
        ROS_ERROR("Param [port] must be given");
        return(-1);
    }
    int imageWidth, imageHeight;
    nh.param<int>("image_width", imageWidth, 320);
    nh.param<int>("image_height", imageHeight, 240);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image", 1);

    cv::VideoCapture cap;
    cap.open(port);
    if (not cap.isOpened())
    {
        ROS_ERROR("Camera %s %s cannot open", nodeName.c_str(), port.c_str());
        return(-1);
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, imageWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, imageHeight);


    cv::Mat frame;
    while (ros::ok())
    {
        cap >> frame;
        if (not frame.empty())
        {
            auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
        }
        else
        {
            ROS_WARN("Camera %s got empty frame", nodeName.c_str());
        }
    }

    cap.release();

    return 0;
}