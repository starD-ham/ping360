//
// Created by hamada on 22/12/06.
//

#include "../include/Ping360_Toscan_Node.h"
Ping360_Toscan_Node::Ping360_Toscan_Node() {

    this -> m_scanPub = m_nh.advertise<sensor_msgs::LaserScan>("/scan",1000);
    this->m_echoSub=m_nh.subscribe("/echo",1000,&Ping360_Toscan_Node::echoCb,this);
    ROS_INFO("Ping360_Toscan_Node is okay");
    m_refstrength_threshold=128;
}

void Ping360_Toscan_Node::echoCb(const Original_msgs::Ping360::ConstPtr &msg) {
    ROS_INFO("Callback");
    std::vector<uint8_t > intensities=msg->intensities;
    int angle_index=msg->angle;

    ROS_INFO("%d",angle_index);
    int index_num =0;
    index_num=findClothObj(intensities, msg->number_of_samples, msg->sample_period);
    ROS_INFO("check");
    this->m_scanMSG.ranges.push_back(index_num*unit_sample_period*1500);
    this->m_scanMSG.intensities.push_back(intensities[index_num]);
    m_scanMSG.angle_increment=0.9;

    if(callback_count==0){//最初のときの処理
        this->scan_start=msg->header.stamp;
        this->m_scanMSG.angle_min=angle_index;
    }
    else if(callback_count==400){//最後のときの処理
        this->m_scanMSG.angle_max=angle_index;
        this->scan_end=msg->header.stamp;
        this->scan_time = scan_end-scan_start;
        m_scanMSG.scan_time=this->scan_time.toSec();

        std::vector<float> finder=this->m_scanMSG.ranges;
        auto it = std::minmax_element(this->m_scanMSG.ranges.begin(), this->m_scanMSG.ranges.end());
        m_scanMSG.range_min= *it.first;
        m_scanMSG.range_max= *it.second;

        m_scanPub.publish(m_scanMSG);
        ROS_INFO("publish scan");
        callback_count=0;
        this->m_scanMSG.ranges.clear();
        this->m_scanMSG.intensities.clear();
    }
    else{

    }
    //ROS_INFO("scan_callback counter is %d",callback_count);
    callback_count++;
}

int Ping360_Toscan_Node::findClothObj(std::vector<uint8_t> intensity, int point_number,double sample_period) {
    double unit_r=sample_period*1500;
    int pinger_area=m_pinger_radius/unit_r;
    for(int i=pinger_area;i<point_number;i++){
        if(intensity[i]>m_refstrength_threshold){
            ROS_INFO("found");
            return i;
        }
    }
    //return 0;

}



void Ping360_Toscan_Node::publishScan(double scan_time) {
    sensor_msgs::LaserScan msg;
    msg.angle_min =0;
    msg.angle_max =400;
    msg.scan_time=scan_time;
    msg.scan_time=scan_time/msg.angle_max;
    msg.header.stamp=ros::Time().now();
    m_scanPub.publish(msg);
}
int main(int argc, char **argv) {
    ros::init(argc,argv,"Toscan_node");
    Ping360_Toscan_Node ping360ToscanNode;
    ros::Rate rate(5);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}