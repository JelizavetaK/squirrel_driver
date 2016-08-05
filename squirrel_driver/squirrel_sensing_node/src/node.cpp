#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "../include/squirrel_sensing_node/node.h"
#include "../include/squirrel_sensing_node/sensing_drivers.h"


using namespace ros;
using namespace std;


const double SensingNode::pause=1000.0;    //this might be became a constructor parameter

//consider instantiating everything in a configure() function instead of the constructor
SensingNode::SensingNode(const std::string& name, const std::string& portname){

    this->name=name;

    tactile_pub=(node.advertise<std_msgs::Float64MultiArray>("fingertips",1));    //1 is maximum number of messages sent before going in overflow
    //proximity_pub=(node.advertise<std_msgs::Float64MultiArray>("pro",1));    //1 is maximum number of messages sent before going in overflow
    wrist_pub=(node.advertise<std_msgs::Float64MultiArray>("wrist",1));    //1 is maximum number of messages sent before going in overflow

    loop_rate=new Rate(pause);

    sensor=new Tactile(portname); //"/dev/ttyACM0");      //the port name will be given from command line
#ifdef _FT17_AVAIL
    wrist=new Wrist();
#else
    wrist=NULL;
#endif
}

SensingNode::~SensingNode(){

    if(sensor!=NULL){
        delete sensor;
    }

    if(wrist!=NULL){
        delete wrist;
    }

    if(loop_rate!=NULL){
        delete loop_rate;
    }

}

void SensingNode::run(){ //this function will make the node loop as long as ros::ok() is true

    //here can be added every function needed to set up the sensor execution (i.e. config file reading)

    sensor->flush();
    while(ros::ok()){
        //read from sensor

        //proximity and tactile
        vector<double>* vals=sensor->readData();

        std_msgs::Float64MultiArray msg;
        msg.data.resize(vals->size());

        //fill in msg
        for(int i=0;i<vals->size();i++){
            msg.data[i]=vals->at(i);
        }

#ifdef _FT17_AVAIL
        //wrist
        vector<double>* wriVals=wrist->readData();

        std_msgs::Float64MultiArray msgWri;
        msgWri.data.resize(wriVals->size());

        //fill in msg
        for(int i=0;i<wriVals->size();i++){
            msgWri.data[i]=wriVals->at(i);
        }
#endif

        //ROS_INFO("Broadcasting...");
        tactile_pub.publish(msg);
#ifdef _FT17_AVAIL
        wrist_pub.publish(msgWri);
#endif

        ros::spinOnce();

        delete vals;

        loop_rate->sleep();
    }

}


string& SensingNode::getName(){
    return name;
}
