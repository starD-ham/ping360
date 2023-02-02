//
// Created by hamada on 22/09/22.
//

#include "../include/pingImage_node.h"
//#include matplotlib-cpp.h

PingImage_node::PingImage_node() {
    this->m_echoSub = this->m_nh.subscribe("/echo",1000,&PingImage_node::echoCb, this);
    this->m_imgPub=this->m_nh.advertise<sensor_msgs::Image>("/image_sonar",1000);
    mat_image = cv::Mat::zeros(cv::Size(x_reso,y_reso),CV_8U);
    publishImage();
    this->state="READY";
    ROS_INFO("%f",this->pi);
}

void PingImage_node::echoCb(const Original_msgs::Ping360::ConstPtr &msg) {
    ROS_INFO("Received echo");
    //active_time=ros::Time::now();
    float linear_factor=msg->intensities.size()/float(center[0]);//中央までのデータ数
    int point_color;// 色の値
    for(int i=0;i<int(center[0]);i++){//中央から端まで色塗りをしていく
        if(i<center[0]){
            point_color=msg->intensities[int(i*linear_factor-1)];//linear_factorに従って色を決める
        }
        else{
            point_color=0;
        }
        for(int k=0;k<2;k++){//先の4角度分を同じ色として色塗りしておく
            float theta=2* this->pi*(msg->angle+k)/400.0+pi;
            float x = float(i)*cos(theta+pi);
            float y = float(i)*sin(theta);
            // 描画　描画位置はズレていたため調整
            mat_image.at<cv::Vec2b>(int(center[0]+x),int(center[1]+y/2+center[1]/2))=point_color;
        }
    }
    if(msg->angle==399){
        cv::imwrite("/home/hamada/catkin_ws/src/ping360/image/"+std::to_string(ros::Time::now().sec)+".jpg",mat_image);
    }
    publishImage();
    state="ACTIVE";
}

void PingImage_node::publishImage() {

    // カラーマップの描画
    cv::applyColorMap(mat_image,jet_image,cv::COLORMAP_JET);//カラーマップ変換
    // 距離を示す円の描画
    cv::circle(jet_image,cv::Point(int(center[0]),int(center[1])),x_reso/2,cv::Scalar(255,255,255),1,cv::LINE_8);
    cv::circle(jet_image,cv::Point(int(center[0]),int(center[1])),x_reso/3,cv::Scalar(255,255,255),1,cv::LINE_8);
    cv::circle(jet_image,cv::Point(int(center[0]),int(center[1])),x_reso/6,cv::Scalar(255,255,255),1,cv::LINE_8);
    bridge.image=jet_image;// ImageMSGに変換するための形式に
    bridge.encoding="bgr8";
    bridge.header.stamp=ros::Time::now();
    this->m_imgPub.publish(bridge.toImageMsg());
    ROS_INFO("published");
    // 角度を表す線の描画
//    cv::line(jet_img,cv::Point(int(center[0])*2/3,0),cv::Point(int(center[0])*2/3,int(2*center[1])*2/3),cv::Scalar(255,255,255));
//    cv::line(jet_img,cv::Point(0,int(center[1])*2/3),cv::Point(int(2*center[0])*2/3,int(center[1])*2/3),cv::Scalar(255,255,255));
//    cv::line(jet_img,cv::Point(int(center[0])*2/3+int(center[0])*2/3*cos(pi/4),int(center[1])+int(center[1]*2/3)*sin(pi/4)),cv::Point(int(center[0]*2/3)-int(center[0])*2/3*cos(pi/4),int(center[1]*2/3)-int(center[1])*2/3*sin(pi/4)),cv::Scalar(255,255,255));
//    cv::line(jet_img,cv::Point(int(center[0])*2/3+int(center[0])*2/3*cos(pi/4),int(center[1])-int(center[1]*2/3)*sin(pi/4)),cv::Point(int(center[0]*2/3)-int(center[0])*2/3*cos(pi/4),int(center[1]*2/3)+int(center[1])*2/3*sin(pi/4)),cv::Scalar(255,255,255));
//    // 角度の文字の描画

    //ガウシアン
//    cv::Mat gaussian;
//    cv::GaussianBlur(jet_img, gaussian, cv::Size(9, 9), 5);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"pingImage_node");
    PingImage_node pingImageNode;
    ros::Rate rate(5);
    while(ros::ok()){
        ROS_INFO_THROTTLE(1, "%s", pingImageNode.state.data());
        if(pingImageNode.state=="ACTIVE"){
            pingImageNode.state="READY";
        }
        ros::spinOnce();
        rate.sleep();
    }
}
