#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include <assert.h>
#include <ros/package.h>
#include <ros/ros.h>


#include "../include/squirrel_sensing_node/sensing_drivers.h"

//#define TEST    //undefine to remove testing code

using namespace std;

const char Driver::CMD_GETDATA[]="g"; //arduino command to get data
const int INVALID_DATA=-100;
const int Driver::MAX_RETRIES=5;   //we try 5 times to read
const double Driver::MAX_VOLTS=5.0;

//makes the connection with m_portname
bool Driver::setup(){    //assuming setup is equal for 2 sensros on 3

    m_fileDesc=open(m_portname.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    fcntl(m_fileDesc, F_SETFL,0);//reset file status flags

    /* Set up the control structure */
     struct termios toptions;
     /* Get currently set options for the tty */
     tcgetattr(m_fileDesc, &toptions);
     /* 9600 baud */
      cfsetispeed(&toptions, B19200);   //enter here B38400 for max baudrate for this system, arduino must match this speed, B9600 was the previous, slowest, value which was proven to work in the larger system
      cfsetospeed(&toptions, B19200);   //enter here B38400 for max baudrate for this system, arduino must match this speed (both lines must have the same speed set here)
      /* 8 bits, no parity, no stop bits */
       toptions.c_cflag &= ~PARENB;
       toptions.c_cflag &= ~CSTOPB;
       toptions.c_cflag &= ~CSIZE;
       toptions.c_cflag |= CS8; //should be one stop bit
       /* Canonical mode */
        toptions.c_lflag |= ICANON;

        //---------------------------------------------------------------
        /*non canonical mode */
       // /* no hardware flow control */
       //toptions.c_cflag &= ~CRTSCTS;
       // /* enable receiver, ignore status lines */
       //toptions.c_cflag |= CREAD | CLOCAL;
       // /* disable input/output flow control, disable restart chars */
       //toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
       // /* disable canonical input, disable echo,
       //disable visually erase chars,
       //disable terminal-generated signals */
       //toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
       // /* disable output processing */
       //toptions.c_oflag &= ~OPOST;
        //---------------------------------------------------------------

    /* commit the options */
     tcsetattr(m_fileDesc, TCSANOW, &toptions);
     /* Flush anything already in the serial buffer */
     tcflush(m_fileDesc, TCIFLUSH);

     return true;
}

void Driver::flush(){
    /* Flush anything already in the serial buffer */
    tcflush(m_fileDesc, TCIFLUSH);
}


RES_COMMS Driver::arduRead(vector<double>& data)
{
    const int CMD_BUF_LEN=3;
    char rpy_buff[255];

    fd_set read_fds, write_fds, except_fds;
    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);    //if we want to check if we can write, we have to add to this set
    FD_ZERO(&except_fds);   //we don't care about exception
    FD_SET(m_fileDesc, &read_fds);  //monitor stream for reading
    // Set timeout to 1.0 seconds
    struct timeval timeout;
    timeout.tv_sec = 1;     //wait for 1.5 seconds
    timeout.tv_usec = 500000;

    tcflush(m_fileDesc, TCIFLUSH);
  //decomment for more reliable communication (request/reply style)

    //write
    ssize_t wrbytes=write(m_fileDesc,CMD_GETDATA,CMD_BUF_LEN);
    if(wrbytes!=CMD_BUF_LEN)  //we couldn't write
    {
        cout << "Driver::arduRead> ERROR! Couldn't issue request to arduino" << endl;
        for(uint i=0;i<NUM_VALS;++i)
        {
            data.push_back(INVALID_DATA);   //return failed data
        }
        return RES_CANNOT_WRITE;
    }


    //read
    ssize_t rdbytes=-2; //termios returns -1 if error
    //"first argument of select is the highest-numbered file descriptor in any of the three sets, plus 1." (cit.)
    if (select(m_fileDesc + 1, &read_fds, &write_fds, &except_fds, &timeout) > 0)   //checks if is possible to read
    {
        rdbytes=read(m_fileDesc,rpy_buff,255);
    }

    if(rdbytes<=0)  //either we couldn't read or there was an error
    {
        cout << "Driver::arduRead> ERROR! Couldn't read anything from arduino or error while reading: "  << (rdbytes==-2? "no data" : "read error") << endl;
        for(uint i=0;i<NUM_VALS;++i)
        {
            data.push_back(INVALID_DATA);
        }
        return RES_CANNOT_READ;
    }
    rpy_buff[rdbytes]='\0'; //now we have a string

    //parse
    stringstream parser(rpy_buff);
    RES_COMMS result=RES_SUCCESS;
    double val=0.0;

    //fascist parser, everything must be correct
    //in nonfascist version we will receive more unreliable data because of hihger speed

    while(parser >> val || !parser.eof())
    {
        if(parser.fail())   //if we received garbage due to loss of sync
        {
            parser.clear(); //clear error
            string alien;
            parser >> alien;
            if(alien!="\n" || alien!="\r")  //arduino sends this all the time
            {
                cout << "WARNING: received surprised: " << alien << endl;
            }
            continue;
        }

        val=val*(5.0 / 1023.0);
        if(val>MAX_VOLTS || val <=0)
        {
            val=INVALID_DATA;
            result=RES_INVALID_DATA; //there was a parsing error
        }
        data.push_back(val); //conversion gain to volts
    }

    /*  //this is commented for mercy
    if(data.size()!=NUM_VALS)
    {
        cout << "WARNING! Unexpected data received" << endl;
        return RES_RECIEVED_GARBAGE;
    }*/


    /*
     * //non-fascist parser: whatever comes in is read for the old arduino code
    for(uint i=0; parser >> val || !parser.eof() ; ++i)
    {
            if(parser.fail())   //if we received garbage due to loss of sync
            {
                parser.clear(); //clear error
                string alien;
                parser >> alien;
                if(alien=="\n" || alien=="\r")  //arduino sends this all the time
                {
                    //if we are here we might have read a full string
                    if(data.size()==NUM_VALS)
                    {
                        break;  //if yes, we done
                    }
                    else
                    {
                        //if not, we are out of sync and our values are garbage, start over again
                        data.clear();
                        cout << "OUT OF SYNC" << endl;
                        continue;
                    }
                }
                cout << "WARNING: received surprised: " << alien << endl;
                continue;
            }

            if(val>MAX_VOLTS || val <=0)    //if you did not read garbage, but you read an impossible value
            {
                cout << "PARSE ERROR: " << val << endl;
                val=INVALID_DATA;
                result=RES_INVALID_DATA; //there was a parsing error

            }

            //if you read a good value
            data.push_back(val);

    }
    */

    return result;

}

//------------------------------TACTILE and PROXIMITY

const int Driver::NUM_TACT=9; //number of tactile values
const int Driver::NUM_PROX=6;
const int Driver::NUM_VALS=Driver::NUM_TACT+Driver::NUM_PROX;  //should be 15

const int Tactile::NUM_HISTORY_VALS=10;
const int NUM_INTIALISATION_VALS=100;
const double Tactile::STATIONARY_TACTILE_THREASHOLD=0.0075;    //volts
const double Tactile::STATIONARY_PROXIMITY_THREASHOLD=0.02;    //volts

//calibration coefficients tactile sensor
const double Tactile::A11_TACT=26.25;
const double Tactile::A12_TACT=21.96;
const double Tactile::A13_TACT=2.608;

const double Tactile::A21_TACT=-2.152;
const double Tactile::A22_TACT=-1.381;
const double Tactile::A23_TACT=0;

const double Tactile::A31_TACT=-12.65;
const double Tactile::A32_TACT=0;
const double Tactile::A33_TACT=27.52;

//maximums of calibration curves in volts for tactile //NEVER set them to 0 or you will get a NaN
const double Tactile::MAX1_V1=0.35;
const double Tactile::MAX1_V2=0.42;
const double Tactile::MAX1_V3=3.56;

const double Tactile::MAX2_V1=0.18;
const double Tactile::MAX2_V2=0.25;
const double Tactile::MAX2_V3=1;

const double Tactile::MAX3_V1=0.15;
const double Tactile::MAX3_V3=-0.12;

//calibration coefficients proximity sensor
const double Tactile::A_PROX=20.74;
const double Tactile::B_PROX=-0.1808; //exponent
const double Tactile::C_PROX=-17.28;

//calibration maximum for proximity
const double Tactile::MAX_PROX=2.5;

//number of values used to calculate average flattening value
const int Tactile::NUM_FLATTENING_TORQUES=10;

std::vector<double> Tactile::m_maximumForce;



Tactile::Tactile(const std::string& portname){

    m_portname=portname;
    m_divider=0;	//we shall never divide by 0
    for(int i=0;i<(3*2);++i)
    {
        torque_perc.push_back(0.0); //3 axis x 2 values set to 0
		m_accumulator_fing.push_back(0.0);	//stores numerators of the mean for each finger and torque sensor
    }

    //sensor is uninitialised at the beginning
    m_isBiased=false;
    m_hasHistoryTact=false;
    m_hasHistoryProx=false;

    //initialising history
    for(int i=0;i<NUM_TACT;i++)
    {
        history_val_tact.push_back(0);
        history_tact.push_back(new queue<double>());
        mean.push_back(vector<double>() );
        m_biases.push_back(0);
        m_lastLegals.push_back(0);
    }
    for(int i=0;i<NUM_PROX;i++)
    {
        history_val_prox.push_back(0);
        history_prox.push_back(new queue<double>());
        mean.push_back(vector<double>() );
        m_biases.push_back(0);
    }


    //read from conf.ini the divider values
    string filepath=ros::package::getPath("squirrel_sensing_node");
    filepath+="/tactile_calibration.ini";
    ifstream config(filepath.c_str());
    bool isReadMaximums=false;  //if more items will be added to the ini file, this will became an enum
    if(config.good()){
        string line;
        int sensId=0;
        while(getline(config,line) && sensId<=NUM_VALS){    //one divider per sensor reading

            if(line.at(0)=='@') //this caracther changes the parsing
            {
              isReadMaximums=true;
            }

            if(!isReadMaximums && line.length()!=0 && line.at(0)!='#' && line.at(0)!='@'){ //if line is not a comment or blank
                istringstream iss(line);
                double val;
                iss >> val;                         //parsing dividers
                divider.push_back(val);
                sensId++;
            }
            else if(isReadMaximums && line.length()!=0 && line.at(0)!='#' && line.at(0)!='@'){ //if line is not a comment or blank
                istringstream iss(line);
                double val;                         //parsing maximums
                iss >> val;
                static int torqueCnt=0;
                if(torqueCnt<6) //cross fingers...
                {
                    maximumTorque.push_back(val);
                }
                else
                {
                    m_maximumForce.push_back(val);
                }
            }
        }

    }else{
        ROS_INFO("ERROR: Could not locate file config.ini, assumed no dividers (==1)");
        for(int i=0;i<NUM_TACT;++i){
            divider.push_back(1);
        }
    }

    setup();
}

Tactile::~Tactile(){

    close(m_fileDesc);

    for(int i=0;i<NUM_TACT;i++)
    {
        delete history_tact[i];
    }

    for(int i=0;i<NUM_PROX;i++)
    {

        delete history_prox[i];
    }
}

//returns true if data is stationary, input is the value and the number of sensor used
bool Tactile::isStationaryTact(const double val,const int idx){

    if(val==INVALID_DATA)   //assume invalid data as stationary
    {
        return false;
    }

    history_tact.at(idx)->push(val);
    history_val_tact.at(idx)+=val;
    if(history_tact.at(idx)->size()<NUM_HISTORY_VALS){ //if we have not enough values yet
        return false;
    }
    m_hasHistoryTact=true;

    history_val_tact.at(idx)-=history_tact.at(idx)->front(); //subtract oldest value
    history_tact.at(idx)->pop();
    return ( (history_val_tact.at(idx)/NUM_HISTORY_VALS) < STATIONARY_TACTILE_THREASHOLD);

}

//returns true if data is stationary, input is the value and the sensor id
//WARNING as it is now, only the first value in a set of 3 is checked for stationarity
bool Tactile::isStationaryProx(const double val,const int idx){

    if(val==INVALID_DATA)   //assume invalid data as stationary
    {
        return false;
    }

    history_prox.at(idx)->push(val);
    history_val_prox.at(idx)+=val;
    if(history_prox.at(idx)->size()<NUM_HISTORY_VALS){ //if we have not enough values yet
        return false;
    }
    m_hasHistoryProx=true;

    history_val_prox.at(idx)-=history_prox.at(idx)->front(); //subtract oldest value
    history_prox.at(idx)->pop();
    return ( (history_val_prox.at(idx)/NUM_HISTORY_VALS) < STATIONARY_PROXIMITY_THREASHOLD);

}

//calculates mean of first 10 values and return the bias value
double Tactile::bias(const int idx,const double val)
{

    if(mean[idx].size()==0){
        mean[idx].push_back(val);
        return 1.0;
    }
    //else calculate mean of past values
    if(mean[idx].size()<NUM_INTIALISATION_VALS){
        mean[idx].push_back(val);
    }

    if(mean[idx].size()==NUM_INTIALISATION_VALS)
    {
        m_isBiased=true;

        //calculate biases
        double accumulator=0.0;
        for(int i=0;i<mean[idx].size();++i){
            accumulator+=mean[idx].at(i);
        }
        m_biases[idx]=(accumulator/mean[idx].size());
    }


    return m_biases[idx];
}

bool Tactile::isSensorInit() const
{

    return ( m_isBiased && m_hasHistoryProx && m_hasHistoryTact );
}

void Tactile::flatteningProcessing(vector<double>& reads)
{

    if(m_divider<NUM_FLATTENING_TORQUES)  //if we have less than NUM_FLATTENING readings, accumulate
    {
        ++m_divider;
		for(int i=0, j=0;i<NUM_TACT;i+=3, j+=2){
			m_accumulator_fing[j]+=reads[i+1];   // 3 is number of values per axis
			m_accumulator_fing[j+1]+=reads[i+2];   // +1 and +2 are to select torque 1 and 2
            //cout << " j " << j << " m_accumulator_fing[j] " << m_accumulator_fing[j] << " m_accumulator_fing[j+1] " << m_accumulator_fing[j+1] << endl;
		}
        //cout << " m_divider " << m_divider << endl;
    }
	else	//else flatten the torque
	{
	    flattenTorque(reads);      //flattens torque values
	}
}

//flatten torque values using the number initialised at startup
void Tactile::flattenTorque(vector<double>& num)
{
	assert(m_divider!=0);	//never divide by 0
	for(int i=0, j=0;i<NUM_TACT;i+=3, j+=2){
		num.at(i+1)-=(m_accumulator_fing[j]/m_divider);    //first torque
		num.at(i+2)-=(m_accumulator_fing[j+1]/m_divider);    //second torque
        //cout << " num.at(i+1) " << num.at(i+1) << " num.at(i+2) " << num.at(i+2) << " mean[j] " << m_accumulator_fing[j]/m_divider << " mean[j+1] " << m_accumulator_fing[j+1]/m_divider << endl;
	}
}

void Tactile::patchData(std::vector<double>& data)
{
    for(int i=0;i<data.size() && i<NUM_TACT;++i)
    {
        if(data[i]==INVALID_DATA)
        {
            data[i]=m_lastLegals[i];
        }
    }

}

void Tactile::updateLegals(std::vector<double>& data)
{

    for(int i=0;i<data.size();++i)
    {
        m_lastLegals[i]=data[i];
    }

}


bool Tactile::readData(std::vector<double>& res)    //we pass a NULL pointer, but we could just give a reference{
{
    res=vector<double>(NUM_VALS,INVALID_DATA); //assume we read only garbage
    bool readRes=false;

    char buff[255];
#ifndef TEST

    vector<double> vals;

    RES_COMMS commsRes=RES_CANNOT_WRITE;
    for(uint i=0;i<MAX_RETRIES && commsRes!=RES_SUCCESS;++i)
    {
        commsRes=arduRead(vals);
        if(commsRes!=RES_SUCCESS){vals.clear();}
    }

    if(commsRes!=RES_SUCCESS && commsRes!=RES_INVALID_DATA )
    {
        cout << "FAIL: Maximum of number of attempts to read from arduino reach, returning failed data" << endl;
    }

    switch(commsRes)
    {
    case RES_CANNOT_WRITE:
    case RES_CANNOT_READ:
    case RES_RECIEVED_GARBAGE:
        return  false;   //reading failed

    case RES_INVALID_DATA:
        patchData(vals);    //if we got an invalid value, we swap it with the last valid value
        readRes=true;
        break;
    case RES_SUCCESS:
        updateLegals(vals);
        readRes=true;   //this reading is usable
        break;
    default:
        break;
    }

#else
    buff[0]='\n';
    cout << "Input volts: " ;
   for(int i=0;i<NUM_VALS;i++)
  {
       res->at(i)=0.1;       //artificial testing volt value
        cout << res->at(i) << " ";
  }
    cout << endl;
   //testing
#endif

    //bias values read from arduino
    for(int i=0, readingNum=0;i<vals.size() && readingNum<NUM_VALS;i++,++readingNum){

        if(vals[i]!=INVALID_DATA)
        {
           res.at(readingNum)=vals[i]-bias(readingNum,vals[i]);  //biasing to zero
        }
        //if an arduino value is invalid, the corresponding value in res would be left as invalid

    }//for stuff in buff


    for(int i=0;i<NUM_TACT;i++){//biasing to calibration
        if(res.at(i)!=INVALID_DATA)
        {
            res.at(i)= res.at(i)*divider[i]; //calibartion curve maximum is (5-1)
        }
    }


    //cout << "Tactile vals: ";
    for(int i=0;i<NUM_TACT;i+=3){//first 9 values are tactile

        if(!isStationaryTact(res.at(i),i)){      //if values changed
          convertTact(res,i);     //we pass 3 values at time (that is why the for increments i+3)
        }else{
            res.at(i)=0;
        }//if not stationary, or invalid


        //cout << res->at(i) << " " << res->at(i+1) << " " << res->at(i+2) << " ";
    }
    //cout << endl ;

    //if still we have invalid data, we set it to 0 so the calculation is not upset
    for(int i=0;i<NUM_TACT;i++)
    {
        if(res.at(i)==INVALID_DATA)
        {
            res.at(i)-0.0;
        }

    }

    //we can flatten the torque only for an initialised sensor
    if(isSensorInit())
    {
        std::vector<double> valuesCopy(res);

        flatteningProcessing(valuesCopy);

        calculateTorquePerc(valuesCopy);  //fills in the torque perc vector
    }
    ///--------------------PROXIMITY CALCULATIONS
    for(int i=NUM_TACT;i<NUM_VALS;i++){//biasing to calibration distance
        res.at(i)= res.at(i)*((divider[i])/MAX_PROX); //calibartion curve maximum is accounted
    }


    for(int i=NUM_TACT;i<NUM_VALS;i++){//last 6 values are proximity

        if(!isStationaryProx(res.at(i),i-NUM_TACT)){      //if values changed
            res.at(i)=convertProx(res.at(i));
        }else{
            res.at(i)=-100; //This is when there is no signal detected
        }

    }


    for(int i=NUM_TACT;i<NUM_VALS;i++)
    {
        if(res[i]!=res[i])  //if this happens, IEEE says we have a nan
        {
            res[i]=m_lastLegals[i];
        }
    }

    ///-----------------------------------------------

    return readRes;
}

//convert volts into newtons
void Tactile::convertTact(vector<double>& num,int idx){

    if(num.size() < idx+2)
    {
        cout << "[Tactile::convertTact] not enough sensor readings to parse in range [" << idx << " " << idx+3 << "] which is larger than maximum number of elements " << num.size() << endl;
        cout << "[Tactile::convertTact] WARNING - Values have NOT been processed!" << endl;
        return; //just to avoid crashes, we could assert it as well
    }

    for(int i=0;i<2;i++)
    {
        if(num[idx+i]==-1) {   //if value not valid
            num[idx+i]=-1;
        }
    }

    double v1=num.at(idx);
    double v2=num.at(idx+1);
    double v3=num.at(idx+2);


    if(num[idx]!=-1)    //these checks are done to be sure that illigal values are not used for computing the formulas
    {
        num[idx]=((A11_TACT/MAX1_V1)*v1)+((A12_TACT/MAX1_V2)*v2)+((A13_TACT/MAX1_V3)*v3);
    }
    if(num[idx+1]!=-1)
    {
        num[idx+1]=((A21_TACT/MAX2_V1)*v1)+((A22_TACT/MAX2_V2)*v2)+((A23_TACT/MAX2_V3)*v3);
    }
    if(num[idx+2]!=-1)
    {
        num[idx+2]=((A31_TACT/MAX3_V1)*v1)+((A33_TACT/MAX3_V3)*v3);
    }

}



//convert volts into distance
double Tactile::convertProx(const double num){

    if(num==INVALID_DATA) {   //if value not valid (shouldn't happen anymore)
        return INVALID_DATA;
    }

    double val=num;   //subtract 1 volt

    return (-((A_PROX*pow(val,B_PROX))+C_PROX));


}

void Tactile::calculateTorquePerc(vector<double>& num)
{

  for(int j=0,i=0;j<9;j+=3,i+=2)    //TODO 9 is the first proximity value
  {
      torque_perc[i]=num.at(j+1);
      torque_perc[i+1]=num.at(j+2);
  }


  for(int i=0;i<torque_perc.size();++i)
  {
      torque_perc[i]=(torque_perc.at(i)/maximumTorque.at(i))*100;
  }

}

//receives three values of force, returns the same values normalised
std::vector<double> Tactile::normaliseForce(const std::vector<double>& force)
{
    if(m_maximumForce.size()==0)   //if we never read, throw an error
    {
        throw runtime_error("Maximum force not initialised yet");
    }
    if(m_maximumForce.size()!=force.size()) //if we screwed up the reading
    {
        throw runtime_error("Numbers of values to normalise do not match number of maximum forces");
    }

    std::vector<double> norms;

    for(int i=0;i<force.size();++i)
    {
        norms.push_back(force.at(i)/m_maximumForce.at(i));
    }

    return norms;
}

vector<double>& Tactile::readTorquePerc()
{
    return torque_perc;
}

//----------------------WRIST-----------------------

Wrist::Wrist(const std::string& portname)
{

    ft17=new FT17Interface ( portname.c_str() );

    ft17->init();
	// FT17 configured in POLLING mode
    ft17->configure_polling ( (uint16_t)127 );
}

Wrist::~Wrist()
{
    //ft17->stop_broadcast();	//it seems is not needed anymore
    delete ft17;
}

bool Wrist::readData(std::vector<double>& res){

	if(ft17==NULL){

        res=vector<double>(WristDataNum,0);
        return false; //WARNING reference to local variable returned

	}

    // get the FT17 data
    //ft17->get_broadcast_data ( ft_bc_data );
	ft17->get_single_data ( ft_bc_data );	//the data structures should be the same but the values require to be checked

    res=vector<double>(WristDataNum,0);



    // fill the FT_filt msg
    //frame_id = std::to_string ( ft_bc_data.board_id ) ; //frame ID, if needed

    res.at(Wrist::ForceX) = ft_bc_data.FT_filt[Wrist::ForceX];
    res.at(Wrist::ForceY) = ft_bc_data.FT_filt[Wrist::ForceY];
    res.at(Wrist::ForceZ) = ft_bc_data.FT_filt[Wrist::ForceZ];
    res.at(Wrist::TorqueX) = ft_bc_data.FT_filt[Wrist::TorqueX];
    res.at(Wrist::TorqueY) = ft_bc_data.FT_filt[Wrist::TorqueY];
    res.at(Wrist::TorqueZ) = ft_bc_data.FT_filt[Wrist::TorqueZ];
    res.at(Wrist::Timestamp) =  ft_bc_data.tStamp ;

    return true;
}

