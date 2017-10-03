#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <exception>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>

#include "../include/squirrel_sensing_node/node.h"
#include "../include/squirrel_sensing_node/sensing_drivers.h"
#include "../include/squirrel_sensing_node/Classificator.h"


using namespace ros;
using namespace std;


// The rate at which the sensors publish. 100 Hz seem enough.
const double SensingNode::pause=100.0;    //this might be became a constructor parameter

//consider instantiating everything in a configure() function instead of the constructor
SensingNode::SensingNode(const std::string& name, const std::vector<std::string>& portnames){

    this->name=name;

    tactile_pub=(node.advertise<std_msgs::Float64MultiArray>("fingertips",1));    //1 is maximum number of messages sent before going in overflow

    wrist_pub=(node.advertise<std_msgs::Float64MultiArray>("wrist",1));    //1 is maximum number of messages sent before going in overflow

    torqPerc_pub=(node.advertise<std_msgs::Float64MultiArray>("torque_percs",1));    //1 is maximum number of messages sent before going in overflow

    m_classification=node.advertise<std_msgs::UInt8MultiArray>("classifier",1);

    for(int i=0;i<FINGERS_NUM;i++)
        dominantTorques.push_back(0.0); //initialise dominant torque vector

    loop_rate=new Rate(pause);

    sensor=new Tactile(portnames[ArduinoPort]);      //the port name is given from command line
#ifdef _FT17_AVAIL
    wrist=new Wrist(portnames[FT17Port]);
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

        //obatin proximity and tactile
        vector<double> vals;
        bool isValid=sensor->readData(vals);

        //obtain torque data
        vector<double> &torqPerc=((Tactile*)sensor)->readTorquePerc();   //obtain percentages
        getTorqueModulo(torqPerc);  //this can be copied blindly in the msg

        //obtain classification
        vector<double> normalTorques(dominantTorques);//copy normalised torque percentages
        for(int i=0;i<normalTorques.size();i++){
            normalTorques.at(i)=normalTorques.at(i)/100;    //convert percentage back to normal value
        }
        //TODO median filter size 2 goes here
        //TODO uncomment once classifier is fully tested
        vector<Decision> overallClass(CLASS_UNDECIDED);//vector<Decision> overallClass=m_stiffClassy.decide(vals,normalTorques);


        //prepare msg for classification
        std_msgs::UInt8MultiArray msgClassy;
        msgClassy.data.resize(overallClass.size());   //number of fingers+1 for overall decision
        //fill in msg for classification
        for(int i=0;i<overallClass.size();i++){
            msgClassy.data[i]=overallClass.at(i);
        }
        //--------


        //prepare msg for force/prox
        std_msgs::Float64MultiArray msgFrPr;
        msgFrPr.data.resize(vals.size());
        //fill in msg for force/proximity
        for(int i=0;i<vals.size();i++){
            msgFrPr.data[i]=vals.at(i);
        }
        //--------

        //prepare msg for torque
                //prox
        vector<double> padProxs;
        padProxs.push_back(vals.at(ProximityPadFing1));    //sensor id 10
        padProxs.push_back(vals.at(ProximityPadFing2));    //sensor id 12
        padProxs.push_back(vals.at(ProximityPadFing3));    //sensor id 14
                //torque
        std_msgs::Float64MultiArray msgTorq;
        msgTorq.data.resize(dominantTorques.size()+padProxs.size());
        //fill in msg for torque
        for(int i=0;i<dominantTorques.size();i++){
            msgTorq.data[i]=dominantTorques.at(i);
        }
        for(int i=dominantTorques.size(), j=0;j<padProxs.size();i++,j++){
            msgTorq.data[i]=padProxs.at(j);
        }
        //--------

#ifdef _FT17_AVAIL
        //wrist
        vector<double> wriVals;

        bool isGood=wrist->readData(wriVals);

        std_msgs::Float64MultiArray msgWri;
        msgWri.data.resize(wriVals.size());

        //fill in msg
        for(int i=0;i<wriVals.size();i++){
            msgWri.data[i]=wriVals.at(i);
        }
#endif

        if(isValid) //publish only if we read valid data
        {
            tactile_pub.publish(msgFrPr);
            torqPerc_pub.publish(msgTorq);
            m_classification.publish(msgClassy);
        }
#ifdef _FT17_AVAIL
        wrist_pub.publish(msgWri);
#endif

        ros::spinOnce();

        loop_rate->sleep();
    }

}

//this function selects the dominant torque, it is not used anymore and it is left as a reference
void SensingNode::pickDominants(vector<double> &torqPerc)
{
    throw runtime_error("Dominant torque not supported, use modulo");   //this is to prevent its use

    for(int i=0,j=0;i<torqPerc.size();i+=2,++j)
    {
        if(torqPerc[i]>torqPerc[i+1])
        {
            dominantTorques[j]=torqPerc[i];
        }
        else
        {
            dominantTorques[j]=torqPerc[i+1];
        }
    }

}
//this function calculates the modulo from two thorques
void SensingNode::getTorqueModulo(vector<double> &torqPerc)
{
    for(int i=0,j=0;i<torqPerc.size();i+=2,++j)
    {
        //modulo
        dominantTorques[j]=sqrt( (pow(torqPerc[i],2)) + (pow(torqPerc[i+1],2)) );
    }

}



string& SensingNode::getName(){
    return name;
}

