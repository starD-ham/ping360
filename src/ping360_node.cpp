//
// Created by hamada on 22/09/13.
//

#include "../include/ping360_node.h"

Ping360_Node::Ping360_Node() {

    this -> m_echoPub = this->m_nh.advertise<Original_msgs::Ping360>("/echo",1000);

    m_serial.setPort("/dev/ttyUSB0");
    m_serial.setBaudrate(115200);
    m_serial.setParity(serial::parity_none);
    m_serial.setStopbits(serial::stopbits_one);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    m_serial.setTimeout(to);
    m_serial.open();
    sleep(1);
    set_default_setting();
}

void Ping360_Node::set_default_setting() {
    m_sensorSettings.mode = 1;
    m_sensorSettings.gain_setting = 0;
    m_sensorSettings.transmit_duration = 5;
    m_sensorSettings.sample_period = 80;
    m_sensorSettings.transmit_frequency = 750;
    m_sensorSettings.num_points = 300;
    m_sensorSettings.transmit =1;
    m_sensorSettings.reserved =0;
}

void Ping360_Node::publishEcho(uint16_t angle,std::vector<unsigned char> intensities) {
    Original_msgs::Ping360 msg;
    msg.header.stamp=ros::Time::now();
    msg.angle=angle;
    msg.gain=m_sensorSettings.gain_setting;
    msg.number_of_samples=m_sensorSettings.num_points;
    msg.sample_period=m_sensorSettings.sample_period*25*10^-9;
    msg.transmit_frequency=m_sensorSettings.transmit_frequency;
    msg.speed_of_sound =0;
    msg.range = 0;
    msg.intensities =intensities;
    m_echoPub.publish(msg);
}

void Ping360_Node::publishScan(double scan_time) {
    sensor_msgs::LaserScan msg;
    msg.angle_min =0;
    msg.angle_max =400;
    msg.scan_time=scan_time;
    msg.scan_time=scan_time/msg.angle_max;
    msg.header.stamp=ros::Time().now();
    m_scanPub.publish(msg);
}

uint16_t* Ping360_Node::control_transducer(uint16_t angle) {
    uint8_t _Wbuffer[24];
    std_msgs::String _Rbuffer;
    uint8_t buf_L, buf_H;// bit演算用の変数
    uint16_t angle_buf=angle;
    uint16_t checkSum=0;
    _Wbuffer[0]=0x42;
    _Wbuffer[1]=0x52;
    _Wbuffer[2]=0x0E;
    _Wbuffer[3]=0x00;
    _Wbuffer[4]=0x29;
    _Wbuffer[5]=0x0A;
    _Wbuffer[6]=0x00;
    _Wbuffer[7]=0x00;
    _Wbuffer[8]=m_sensorSettings.mode;
    _Wbuffer[9]=m_sensorSettings.gain_setting;
    buf_L = angle_buf&0x00ff;
    buf_H = angle_buf>>8;
    _Wbuffer[10]=buf_L;
    _Wbuffer[11]=buf_H;
    buf_L = m_sensorSettings.transmit_duration&0x00ff;
    buf_H = m_sensorSettings.transmit_duration>>8;
    _Wbuffer[12]=buf_L;
    _Wbuffer[13]=buf_H;
    buf_L = m_sensorSettings.sample_period&0x00ff;
    buf_H = m_sensorSettings.sample_period>>8;
    _Wbuffer[14]=buf_L;
    _Wbuffer[15]=buf_H;
    buf_L = m_sensorSettings.transmit_frequency&0x00ff;
    buf_H = m_sensorSettings.transmit_frequency>>8;
    _Wbuffer[16]=buf_L;
    _Wbuffer[17]=buf_H;
    buf_L = m_sensorSettings.num_points&0x00ff;
    buf_H = m_sensorSettings.num_points>>8;
    _Wbuffer[18]=buf_L;
    _Wbuffer[19]=buf_H;
    _Wbuffer[20]=m_sensorSettings.transmit;
    _Wbuffer[21]=0;
    for(int i=0;i<=21;i++){
        checkSum+=_Wbuffer[i];
        printf("%d",_Wbuffer[i]);
    }
    buf_L = checkSum&0x00ff;
    buf_H = checkSum>>8;
    _Wbuffer[22]=buf_L;
    _Wbuffer[23]=buf_H;
    m_serial.write(_Wbuffer,24);
    m_serial.waitReadable();
    ros::Rate rate(20);
    rate.sleep();

    if(m_serial.available()){
        _Rbuffer.data = m_serial.read(m_serial.available());
        std::vector<unsigned char> _buf_char(_Rbuffer.data.begin(),_Rbuffer.data.end());
        std::vector<unsigned char> _intensity;
        if(m_sensorSettings.num_points==(_buf_char.size()-24)){//前に22個、後ろに2個通信用のデータがくっついてくる
            for(int i=22;i<m_sensorSettings.num_points+22;i++){
                _intensity.push_back(_buf_char.at(i));
            }
            std::cout << "Same size"<< std::endl;
        }
        else{
            std::cout << "Not same size"<< std::endl;
            std::cout<<_buf_char.size()<<std::endl;
            std::cout<<m_sensorSettings.num_points<<std::endl;
        }
//        std::cout << "Rbuffer is" << _Rbuffer.data << std::endl;
//        uint8_t buf[m_sensorSettings.num_points];
//        int n =0;
//        for(int i=_buf_char[24];i<=m_sensorSettings.num_points;i++){
//            buf[n] = static_cast<uint8_t>(_buf_char.at(i));
//            n ++;
//        }

        publishEcho(angle,_intensity);
        //ROS_INFO("published");
    }
    else{
        //printf("not available tranceducer!!");
    }
    for(int i = 24;i<=24+m_sensorSettings.num_points;i++){
        //data(i)=buf_uchar(i);
    }
    return 0;
}

uint16_t Ping360_Node::concatData(unsigned char data1, unsigned char data2) {
    uint16_t concatenated_data = 0;
    concatenated_data =data1;
    concatenated_data |= data2 << 8;
    return concatenated_data;
}

int main(int argc, char **argv) {

    ros::init(argc,argv,"ping360_node");
    Ping360_Node ping360Node;


    if(ping360Node.m_serial.isOpen()){

    }
    else{
        return -1;
    }
    ping360Node.m_serial.waitReadable();
    uint8_t Wbuffer[12] = {0};
    std_msgs::String Rbuffer;
    uint8_t buf_L, buf_H;// bit演算用の変数
    uint16_t checkSum = 0;
    Wbuffer[0] = 0x42;
    Wbuffer[1] = 0x52;
    Wbuffer[2] = 0x02;
    Wbuffer[3] = 0x00;
    Wbuffer[4] = 0x06;
    Wbuffer[5] = 0x00;
    Wbuffer[6] = 0x00;
    Wbuffer[7] = 0x00;
    Wbuffer[8] = 0x05;
    Wbuffer[9] = 0x00;
    for (int i = 0; i <= 9; i++) {
        checkSum += Wbuffer[i];
    }
    buf_L = checkSum & 0x00ff;
    buf_H = checkSum >> 8;
    Wbuffer[10] = buf_L;
    Wbuffer[11] = buf_H;

    //std::cout<<"Rbuffer is"<<buf<<std::endl;
    ping360Node.m_serial.write(Wbuffer, 12);
    ping360Node.m_serial.waitReadable();
    if(ping360Node.m_serial.available()){
        Rbuffer.data = ping360Node.m_serial.read(ping360Node.m_serial.available());
        std::vector<unsigned char> buf_char(Rbuffer.data.begin(),Rbuffer.data.end());
        std::cout << "Rbuffer is"<<Rbuffer.data << std::endl;
    }
    else{
        printf("not available protocol!!");
        return -1;
    }
    //ping360Node.control_transducer(1);
    ros::Rate rate(20);
    ros::Time time;
    double start_time, end_time;
    double scan_time;
    while(ros::ok()){
        for(int i=0;i<400;i++) {
            ping360Node.control_transducer(i);
        }

        ros::spinOnce();
        rate.sleep();
    }
    ping360Node.m_serial.close();
}