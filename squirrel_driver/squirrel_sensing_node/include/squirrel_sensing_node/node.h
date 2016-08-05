#ifndef SENSING_NODE
#define SENSING_NODE

#include <string>

#include "ros/ros.h"
#include "sensing_drivers.h"

//comment below line to disable wrist sensor (in case it is not plugged in)
//#define _FT17_AVAIL


class SensingNode{

    static const double pause;   //period of operation (human-readable)

    ros::NodeHandle node;   //ros node
    ros::Publisher tactile_pub;  //publisher
    ros::Publisher proximity_pub;  //publisher
    ros::Publisher wrist_pub;  //publisher
    ros::Rate* loop_rate;    //execution frequency (for ros)

    Driver* sensor; //this variable is the concrete sensor being used
    Driver* wrist;

    std::string name;

public:
    SensingNode(const std::string& name, const std::string& portname);
    ~SensingNode();

    void run(); //this function will make the node loop as long as ros::ok() is true

    //getters and setters
    std::string& getName();
};





#endif
