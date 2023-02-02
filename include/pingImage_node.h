//
// Created by hamada on 22/09/22.
//

#ifndef PING_CPP_PINGIMAGE_NODE_H
#define PING_CPP_PINGIMAGE_NODE_H
#include "ros/ros.h"
#include "Original_msgs/Ping360.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "cv_bridge/cv_bridge.h"
#include "math.h"

class PingImage_node {
public:
    PingImage_node();

    ros::NodeHandle m_nh;
    ros::Subscriber m_echoSub;
    ros::Publisher m_imgPub;
    void echoCb(const Original_msgs::Ping360::ConstPtr& msg);

    cv_bridge::CvImage bridge;
    cv::Mat mat_image;
    void publishImage();
    const int x_reso=4*2*50;
    const int y_reso=4*2*50;
    const double pi = 2*acos(0.0);
    float center[2]={float(x_reso/2),float(y_reso/2)};
    std::vector<float> deg;
    std::vector<float> r_dist;

    sensor_msgs::Image m_image;

};


#endif //PING_CPP_PINGIMAGE_NODE_H
