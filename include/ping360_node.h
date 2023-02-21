//
// Created by hamada on 22/09/13.
//

#ifndef PING_CPP_PING360_NODE_H
#define PING_CPP_PING360_NODE_H

#include "ros/ros.h"
#include "serial/serial.h"
#include <std_msgs/String.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "Original_msgs/Ping360.h"

// dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>



class Ping360_Node {

public:
    Ping360_Node();

    ros::NodeHandle m_nh;
    ros::Publisher m_echoPub;
    ros::Publisher m_scanPub;
    void publishEcho(uint16_t angle, std::vector<signed char> intensities);
    void publishScan(double scan_time);

    serial::Serial m_serial;
    sensor_msgs::Image m_image;
    sensor_msgs::LaserScan m_scan;
    Original_msgs::Ping360 m_echo;

    void set_sample_period(uint16_t sample_period){
        if (m_sensorSettings.sample_period != sample_period){
            m_sensorSettings.sample_period =sample_period;
        }
    }
    int sample_period() const { return m_sensorSettings.sample_period; }

    void set_transmit_frequency(int transmit_frequency)
    {
        m_sensorSettings.transmit_frequency = transmit_frequency;
    }
    int transmit_frequency() const { return m_sensorSettings.transmit_frequency; }

    void set_gain_setting(int gain_setting)
    {
        m_sensorSettings.gain_setting = gain_setting;
    }
    uint32_t gain_setting() const { return m_sensorSettings.gain_setting; }

    void set_default_setting();

    uint16_t* control_transducer(uint16_t angle);
    uint16_t concatData(unsigned char data1, unsigned char data2);

    struct Ping360Setting{
        int mode;
        int transmit_duration;
        int gain_setting;
        int num_points;
        int sample_period;
        int transmit_frequency;
        int transmit;
        int reserved;
        int start_angle = 0;
        int end_angle = 0;
    } m_sensorSettings;

    void sonarParamInit(ros::NodeHandle &pnh);
    void setDynamicParam(std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>,const int,const int);
};


#endif //PING_CPP_PING360_NODE_H
