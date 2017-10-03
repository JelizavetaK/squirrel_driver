#ifndef SENSING_DRIVERS
#define SENSING_DRIVERS

#include <string>
#include <vector>
#include <queue>
#include <FT17/FT17Interface.h>
#include "common_defines.h"



class Driver{   //abstract class: everything in common across sensors is here
//nothing here (no private data)
protected:
    std::string m_portname; //name of the arduino port
    int m_fileDesc;         //file descriptor for termios
    std::vector<double> m_sensor_values;

    static const int NUM_TACT; //num of tactile sensors
    static const int NUM_PROX; //num of proximity sensors
    static const int NUM_VALS; //num of readings
    static const double MAX_VOLTS;  //beyond this value does not make sense
    static const int MAX_RETRIES;

    //config matrix (filename?)

    virtual bool setup();   //assuming setup is equal for 2 sensors on 3


public:
    virtual bool readData(std::vector<double>&)=0;  //this function reads the data from the sensors and returns a vector (double[][])
    virtual void flush();
    RES_COMMS arduRead(std::vector<double>& data);

private:
    static const char CMD_GETDATA[5];  //command to fetch data from arduino


};

//------------------------------


class Tactile : public Driver{

    // calibration coefficients
    static const double A11_TACT;
    static const double A12_TACT;
    static const double A13_TACT;

    static const double A21_TACT;
    static const double A22_TACT;
    static const double A23_TACT;

    static const double A31_TACT;
    static const double A32_TACT;
    static const double A33_TACT;

    //--- maximums
    static const double MAX1_V1;
    static const double MAX1_V2;
    static const double MAX1_V3;

    static const double MAX2_V1;
    static const double MAX2_V2;
    static const double MAX2_V3;

    static const double MAX3_V1;
    static const double MAX3_V3;
    //---
    static const double MAX_PROX; //proximity

    static const double A_PROX;
    static const double B_PROX;
    static const double C_PROX;
    static const double D_PROX;

    static const int NUM_HISTORY_VALS; //number of readings to use for stationary check
    static const double STATIONARY_TACTILE_THREASHOLD; //voltage threashold, if passed data is stationary
    static const double STATIONARY_PROXIMITY_THREASHOLD; //voltage threashold, if passed data is stationary
    static const int NUM_FLATTENING_TORQUES; //number of values to be used for calculating the mean for flattening the torque

    std::vector<double> divider; //vecotr containing voltage divider numbers read from file
    std::vector<double> history_val_tact; //sum previous voltage values
    std::vector<std::queue<double>* > history_tact; //list of previous voltage values
    std::vector<double> history_val_prox; //sum previous voltage values
    std::vector<std::queue<double>* > history_prox; //list of previous voltage values
    std::vector<std::vector<double> > mean; //first 10 values used for calculating the bias
    std::vector<double> maximumTorque;  //vector containing the maximum toque values for tactile sensor
    std::vector<double> torque_perc;    //pecrentages of torque values
	std::vector<double> m_accumulator_fing;	//numerators of the mean
    std::vector<double> m_biases;   //biases for the arduino readings, 1 value per sensor
    std::vector<double> m_lastLegals;   //list of last legal values
    static std::vector<double> m_maximumForce;  //vector containing the maximum force values for each tactile sensor (3x1) //made static for brevity

    int m_divider;              //counts the number of samples read for flattening the torque
    //those three guys are used to discriminate which components of the driver are initialised
    bool m_isBiased;
    bool m_hasHistoryTact;
    bool m_hasHistoryProx;

    double bias(const int idx,const double val);
    void convertTact(std::vector<double>& num,int idx);
    double convertProx(const double num);
    bool isStationaryTact(const double val,const int idx);
    bool isStationaryProx(const double val,const int idx);
    void flatteningProcessing(std::vector<double>& num);    //calculates accumulator and dividers to flatten the torques
    void flattenTorque(std::vector<double>& num);
    void calculateTorquePerc(std::vector<double>& num);
    void patchData(std::vector<double>& data);
    void updateLegals(std::vector<double>& data);

public:
    Tactile(const std::string& portname);
    ~Tactile();
    virtual bool readData(std::vector<double>&);
    std::vector<double>& readTorquePerc();
    bool isSensorInit() const;  //returns true if all the components of the sensor are initialised

    //normalises a force reading, throws runtime error if cannot be done yet
    static std::vector<double> normaliseForce(const std::vector<double>& force);
};


class Wrist : public Driver{

    ft_data ft_bc_data;
    FT17Interface* ft17;
    std::vector<double> m_sensor_values;
public:
    enum WristData
    {
        ForceX,
        ForceY,
        ForceZ,
        TorqueX,
        TorqueY,
        TorqueZ,
        Timestamp,
        WristDataNum
    };

    Wrist(const std::string& portname);
    ~Wrist();

    virtual bool readData(std::vector<double>&);

};

#endif
