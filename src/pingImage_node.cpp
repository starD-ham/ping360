//
// Created by hamada on 22/09/22.
//

#include "../include/pingImage_node.h"
//#include matplotlib-cpp.h

PingImage_node::PingImage_node() {
    this->m_echoSub = this->m_nh.subscribe("/echo",1000,&PingImage_node::echoCb, this);
    this->m_sonar_gray_imgPub=this->m_nh.advertise<sensor_msgs::Image>("/sonar_img/raw",1000);
    this->m_sonar_jet_imgPub=this->m_nh.advertise<sensor_msgs::Image>("/sonar_img/jet",1000);
    this->m_sonar_data_Pub=this->m_nh.advertise<sensor_msgs::Image>("/sonar_img/data_mat",1000);
    mat_image = cv::Mat::zeros(cv::Size(x_reso,y_reso),CV_8U);
    data_image = cv::Mat::zeros(cv::Size(x_reso,y_reso),CV_8U);
    //publishImage();
}

void PingImage_node::echoCb(const Original_msgs::Ping360::ConstPtr &msg) {
    ROS_INFO("callback");
    if(msg->intensities.empty()) {
        ROS_ERROR("Empty intensities");
        return;
    }
    float linear_factor=msg->intensities.size()/float(center[0]);//中央までのデータ数
    std::cout<<linear_factor<<std::endl;
    int point_color;// 色の値

    for(int i=0;i<int(center[0]);i++){//中央から端まで色塗りをしていく
        if(i<center[0]){
            point_color=msg->intensities[int(i*linear_factor-1)];//
        }
        else{
            point_color=0;
        }
        for(double k=0;k<20;k++){//みやすさのため先の4角度分を同じ色として色塗りしておく
            double theta=2* this->pi*(msg->angle+k/10)/400.0;
            double x = double(i)*cos(theta);
            double y = double(i)*sin(theta+pi);


            // 描画　描画位置はズレていたため調整
            mat_image.at<cv::Vec2b>(int(center[0]+x),int(center[1]+y/2+center[1]/2))=point_color;
            if(k==0){
                data_image.at<cv::Vec2b>(int(center[0]+x),int(center[1]+y/2+center[1]/2))=255;
            }

//            for(int mm=-4;mm<5;mm++){
//                for(int nn=-4;nn<5;nn++){
//                    mat_image.at<cv::Vec2b>(int(center[0]+x+mm),int(center[1]+y/2+center[1]/2)+nn)=point_color;
//                }
//            }
        }
    }
    publishImage(2*pi*msg->angle/400.0);
    if(msg->angle>398){
        //cv::imwrite("/home/hamada/catkin_ws/src/ping360/image/"+ std::to_string(msg->header.stamp.toSec())+".jpg",mat_image);
    }
}

void PingImage_node::publishImage(float angle) {


    // カラーマップの描画
    cv::Mat jet_img,gray_img;
    cv::applyColorMap(mat_image,jet_img,cv::COLORMAP_JET);
    //cv::cvtColor(mat_image,gray_img,cv::COLOR_GRAY);
    gray_img=mat_image.clone();

    double x = 1200*sin(angle+pi)+center[0];
    double y = -1200*cos(angle+pi)+center[1];
    cv::line(jet_img,cv::Point(center[0],center[1]),cv::Point(x,y),cv::Scalar(255,255,255),1,cv::LINE_4);
//    // 距離を示す円の描画
//    cv::circle(gray_img,cv::Point(int(center[0]),int(center[1])),x_reso/2,cv::Scalar(255,255,255),0,cv::LINE_8);
//    cv::circle(gray_img,cv::Point(int(center[0]),int(center[1])),x_reso/3,cv::Scalar(255,255,255),0,cv::LINE_8);
//    cv::circle(gray_img,cv::Point(int(center[0]),int(center[1])),x_reso/6,cv::Scalar(255,255,255),0,cv::LINE_8);
//    // カメラ撮影範囲の描画
//    float camera_angle=(96.1*pi*2)/(4*180*2);//カメラの水中での画角 TODO　要検証
//    float x_diff=100/1.5;//カメラとソナーのx位置 0.1m*1000[mm/m]*/1.5[1ピクセル/mm]
//    float y_diff=0;//カメラとソナーのy位置
//    cv::line(gray_img,cv::Point(int(center[0]-x_diff),int(center[1])),cv::Point(int(center[0]+center[1]*tan(camera_angle)-x_diff),0),cv::Scalar(255,255,255),1,cv::LINE_4);
//    cv::line(gray_img,cv::Point(int(center[0]-x_diff),int(center[1])),cv::Point(int(center[0]+center[1]*tan(-camera_angle)-x_diff),0),cv::Scalar(255,255,255),1,cv::LINE_4);



    bridge_jet.image=jet_img;
    bridge_jet.encoding="bgr8";
    bridge_jet.header.stamp=ros::Time::now();

    bridge_original.image=gray_img;
    bridge_original.encoding="mono8";
    bridge_original.header.stamp=ros::Time::now();

    bridge_data.image=data_image;
    bridge_data.encoding="mono8";
    bridge_data.header.stamp=ros::Time::now();

    this->m_sonar_gray_imgPub.publish(bridge_original.toImageMsg());
    this->m_sonar_jet_imgPub.publish(bridge_jet.toImageMsg());
    this->m_sonar_data_Pub.publish(bridge_data.toImageMsg());

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
