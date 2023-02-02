//
// Created by hamada on 22/09/22.
//

#include "../include/pingImage_node.h"
//#include matplotlib-cpp.h

PingImage_node::PingImage_node() {
    this->m_echoSub = this->m_nh.subscribe("/echo",1000,&PingImage_node::echoCb, this);
    this->m_imgPub=this->m_nh.advertise<sensor_msgs::Image>("/image_debug",1000);
    mat_image = cv::Mat::zeros(cv::Size(x_reso,y_reso),CV_8U);
    publishImage();
}

void PingImage_node::echoCb(const Original_msgs::Ping360::ConstPtr &msg) {
    ROS_INFO("callback");
    float linear_factor=msg->intensities.size()/float(center[0]);//中央までのデータ数
    int point_color;// 色の値

    for(int i=0;i<int(center[0]);i++){//中央から端まで色塗りをしていく
        if(i<center[0]){
            point_color=msg->intensities[int(i*linear_factor-1)];//
        }
        else{
            point_color=0;
        }
        for(int k=0;k<2;k++){//先の4角度分を同じ色として色塗りしておく
            float theta=2* this->pi*(msg->angle+k)/400.0;
            float x = float(i)*cos(theta);
            float y = float(i)*sin(theta);

            // 描画　描画位置はズレていたため調整
            mat_image.at<cv::Vec2b>(int(center[0]+x),int(center[1]+y/2+center[1]/2))=point_color;
        }
    }
    publishImage();
}

void PingImage_node::publishImage() {


    // カラーマップの描画
    cv::Mat jet_img;
    cv::applyColorMap(mat_image,jet_img,cv::COLORMAP_JET);

    // 距離を示す円の描画
    cv::circle(jet_img,cv::Point(int(center[0]),int(center[1])),x_reso/2,cv::Scalar(255,255,255),0,cv::LINE_8);
    cv::circle(jet_img,cv::Point(int(center[0]),int(center[1])),x_reso/3,cv::Scalar(255,255,255),0,cv::LINE_8);
    cv::circle(jet_img,cv::Point(int(center[0]),int(center[1])),x_reso/6,cv::Scalar(255,255,255),0,cv::LINE_8);
    // カメラ撮影範囲の描画
    float camera_angle=96.1*3/4/180*3.14;//カメラの水中での画角
    float x_diff=0.1*x_reso/1.5;//カメラとソナーのx位置 TODO:ピクセル数に変換する
    float y_diff=0;//カメラとソナーのy位置
    //cv::line(jet_img,cv::Point(int(center[0]-x_diff),int(center[1])),cv::Point(int(center[0]+center[1]*tan(camera_angle)-x_diff),0),cv::Scalar(255,255,255),2,cv::LINE_4);
    //cv::line(jet_img,cv::Point(int(center[0]-x_diff),int(center[1])),cv::Point(int(center[0]+center[1]*tan(-camera_angle)-x_diff),0),cv::Scalar(255,255,255),2,cv::LINE_4);

    bridge.image=jet_img;
    bridge.encoding="bgr8";
    bridge.header.stamp=ros::Time::now();


    this->m_imgPub.publish(bridge.toImageMsg());
    ROS_INFO("publish now");
}

int main(int argc, char **argv){
    ros::init(argc,argv,"pingImage_node");
    PingImage_node pingImageNode;
    ros::Rate rate(20);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

}
