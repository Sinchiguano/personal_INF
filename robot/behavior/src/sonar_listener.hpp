#include "ros/ros.h"
//#include "std_msgs/Int16.h"
#include "sonars/SonarMsg.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>


class SonarListener {

    int16_t dist_;
    int16_t z_;
    bool new_data_ = false;

    public:
        SonarListener() { std::cout << "made a SonarListener obj\n"; }
        //void SonarCB(const std_msgs::Int16::ConstPtr&);
        void SonarCB(const sonars::SonarMsg::ConstPtr&);
        std::vector<int16_t> getMsg();
        bool getNewDataBool();

};

//void SonarListener::SonarCB(const std_msgs::Int16::ConstPtr& msg) {
void SonarListener::SonarCB(const sonars::SonarMsg::ConstPtr& msg) {
    dist_ = msg->data;
    z_ = msg->z;
    new_data_ = true;
    //std::cout << "Got " << msg->data << std::endl;
}

std::vector<int16_t> SonarListener::getMsg() {
    new_data_ = false;
    std::vector<int16_t> ret;
    ret.push_back(dist_);
    ret.push_back(z_);
    //std::cout << "returning msg " << dist_ << std::endl;
    return ret;
}

bool SonarListener::getNewDataBool() {
    return new_data_;
}


