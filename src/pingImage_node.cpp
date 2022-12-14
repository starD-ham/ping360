//
// Created by hamada on 22/09/22.
//

#include "../include/pingImage_node.h"
//#include matplotlib-cpp.h

PingImage_node::PingImage_node() {
    this->m_echoSub = this->m_nh.subscribe("/echo",1000,&PingImage_node::echoCb, this);
    this->m_imgPub=this->m_nh.advertise<sensor_msgs::Image>("/image",1000);
    mat_image = cv::Mat::zeros(cv::Size(x_reso,y_reso),CV_8U);

    publishImage();
    this->state="READY";
}

void PingImage_node::echoCb(const Original_msgs::Ping360::ConstPtr &msg) {
    active_time=ros::Time::now();
    float linear_factor=msg->intensities.size()/float(center[0]);//中央までのデータ数
    int point_color;// 色の値

    for(int i=0;i<int(center[0]);i++){//中央から端まで色塗りをしていく
        if(i<center[0]){
            point_color=msg->intensities[int(i*linear_factor-1)];//
        }
        else{
            point_color=0;
        }
        if(point_color<0){
            point_color=0;
        }
        for(int k=0;k<1;k++){//先の4角度分を同じ色として色塗りしておく
            float theta=2* this->pi*(msg->angle+k)/400.0;
            float x = float(i)*cos(theta);
            float y = float(i)*sin(theta);

            // 描画　描画位置はズレていたため調整
            mat_image.at<cv::Vec2b>(int(center[0]+x),int(center[1]+y/2+center[1]/2))=point_color;
        }

        publishImage();
    }
    state="ACTIVE";
}

void PingImage_node::publishImage() {


    // カラーマップの描画
    cv::Mat jet_img;
    cv::applyColorMap(mat_image,jet_img,cv::COLORMAP_JET);
    // 距離を示す円の描画
    cv::circle(jet_img,cv::Point(int(center[0]),int(center[1])),x_reso/2,cv::Scalar(255,255,255));
    cv::circle(jet_img,cv::Point(int(center[0]),int(center[1])),x_reso/3,cv::Scalar(255,255,255));
    cv::circle(jet_img,cv::Point(int(center[0]),int(center[1])),x_reso/6,cv::Scalar(255,255,255));
    // 角度を表す線の描画
    cv::line(jet_img,cv::Point(int(center[0]),0),cv::Point(int(center[0]),int(2*center[1])),cv::Scalar(255,255,255));
    cv::line(jet_img,cv::Point(0,int(center[1])),cv::Point(int(2*center[0]),int(center[1])),cv::Scalar(255,255,255));
    cv::line(jet_img,cv::Point(int(center[0])+int(center[0])*cos(pi/4),int(center[1])+int(center[1])*sin(pi/4)),cv::Point(int(center[0])-int(center[0])*cos(pi/4),int(center[1])-int(center[1])*sin(pi/4)),cv::Scalar(255,255,255));
    cv::line(jet_img,cv::Point(int(center[0])+int(center[0])*cos(pi/4),int(center[1])-int(center[1])*sin(pi/4)),cv::Point(int(center[0])-int(center[0])*cos(pi/4),int(center[1])+int(center[1])*sin(pi/4)),cv::Scalar(255,255,255));

    //ガウシアン
    cv::Mat gaussian;
    cv::GaussianBlur(jet_img, gaussian, cv::Size(9, 9), 5);
    bridge.image=jet_img;
    bridge.encoding="bgr8";
    bridge.header.stamp=ros::Time::now();

    this->m_imgPub.publish(bridge.toImageMsg());
    ROS_INFO("%d",ros::Time::now().sec);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"pingImage_node");
    PingImage_node pingImageNode;
    ros::Rate rate(5);
    while(ros::ok()){

        ROS_INFO_THROTTLE(1, "%s", pingImageNode.state.data());

        ros::spinOnce();
        rate.sleep();
    }

}
