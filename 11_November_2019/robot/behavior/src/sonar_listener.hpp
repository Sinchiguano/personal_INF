#include "ros/ros.h"
#include "std_msgs/Int16.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>


class SonarListener {

    int16_t dist_;
    bool new_data_ = false;

    public:
        SonarListener() { std::cout << "made a SonarListener obj\n"; }
        void SonarCB(const std_msgs::Int16::ConstPtr&);
        int16_t getMsg();
        bool getNewDataBool();

};

void SonarListener::SonarCB(const std_msgs::Int16::ConstPtr& msg) {
    dist_ = msg->data;
    new_data_ = true;
    //std::cout << "Got " << msg->data << std::endl;
}

int16_t SonarListener::getMsg() {
    new_data_ = false;
    //std::cout << "returning msg " << dist_ << std::endl;
    return dist_;
}

bool SonarListener::getNewDataBool() {
    return new_data_;
}


