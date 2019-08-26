#include <iostream>

#include <ros/ros.h>

#include "move_dynamixel.hpp"

int main(int argc, char** argv) {
    jik::DynamixelCmd dyn;
    if(!dyn.connect("/dev/ttyUSB1", 57600, 1)) return 1;
    if(!dyn.init()) return 1;
    if(!dyn.setGain(100, 800, 0)) return 1;

    ros::init(argc, argv, "move_dynamixel");

    ros::NodeHandle n;

    ros::Rate rate(100.0);

    int j = 0;
    bool rev = false;

    while(ros::ok()) {
        ros::spinOnce();
        if(!dyn.setGoalPos_int(j)) {
            ROS_ERROR("!!!!");
        }
        std::cout << dyn.currPos_int() << std::endl;

        if(rev) j -= 30;
        else j += 30;
        if(j <= 0) {
            j = 0;
            rev = false;
        }
        else if(j >= 4001) {
            j = 4000;
            rev = true;
        }

        rate.sleep();
    }
    
    dyn.disconnect();
    
    return 0;
}
