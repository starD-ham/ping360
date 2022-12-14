//
// Created by hamada on 22/12/06.
//

#ifndef PING_CPP_PING360_TOSCAN_NODE_H
#define PING_CPP_PING360_TOSCAN_NODE_H
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "Original_msgs/Ping360.h"

class Ping360_Toscan_Node {

public:
    Ping360_Toscan_Node();

    ros::NodeHandle m_nh;
    ros::Subscriber m_echoSub;
    ros::Publisher m_scanPub;
    void echoCb(const Original_msgs::Ping360::ConstPtr& msg);
    int callback_count=0;
    int findClothObj(std::vector<uint8_t> intensity, int point_number,double sample_period);
    double m_refstrength_threshold;
    double m_pinger_radius=0.1;
    const double unit_sample_period = 25*10^-9;

    void publishScan(double scan_time);
    sensor_msgs::LaserScan m_scanMSG;
    ros::Time scan_start;
    ros::Time scan_end;
    ros::Duration scan_time;


};


#endif //PING_CPP_PING360_TOSCAN_NODE_H
