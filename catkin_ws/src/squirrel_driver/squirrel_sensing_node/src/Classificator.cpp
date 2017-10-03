#include "../include/squirrel_sensing_node/Classificator.h"
#include "../include/squirrel_sensing_node/common_defines.h"
#include "../include/squirrel_sensing_node/sensing_drivers.h"
#include <cassert>
#include <iostream>


//TODO normalise force
//TODO accumulated standard deviation of normalised force (use the same threashold of sensing_drivers)
//TODO implement filter before sending data to classifier

const double Classificator::FORCE_PAR =-5.2629;  //a F +
const double Classificator::TORQUE_PAR=-2.9108;  //b T +
const double Classificator::FREE_PAR  = 2.38;    //c

//if a value falls withing this threashold (+/-) it is ambiguos
const double Classificator::AMBIGUITY_THREAS=0.2;    //TODO this might require adjustments
const double Classificator::PROX_TRIGGER=-2;    //anything bigger than this triggers a classification

Classificator::Classificator() : m_calibrated(false)
{
    for(int i=0;i<FINGERS_NUM;++i)
    {
        m_classifications.push_back(CLASS_UNDECIDED);
    }

    for(int i=0;i<FeaturesNum;++i)
    {
        std::vector<std::deque<double> > tmpfeat;
        m_meansHistory.push_back(tmpfeat);
        std::vector<double> tmpmeans(FINGERS_NUM,0.0);
        m_means.push_back(tmpmeans);    //a vector per feature, each vector has as many values as sensors
        for(int j=0;j<FINGERS_NUM;++j)
        {
            std::deque<double> tmpfing;
            m_meansHistory.at(i).push_back(tmpfing);
            //values are added as they come in
        }
    }
}


Decision Classificator::svm(double fr, double tq)
{
    Decision classy=CLASS_UNDECIDED;

    double detection=(FORCE_PAR*fr)+(TORQUE_PAR*tq)+FREE_PAR;

    if( detection>0 )    //if it looks hard
    {
        //if is not within the ambiguity area, then is really hard
        classy=(detection-AMBIGUITY_THREAS) > 0 ? CLASS_HARD : CLASS_SOFT;
    }
    else    //if negative (hence soft)
    {
        classy=CLASS_SOFT;
    }

    return classy;
}

// calculate standard deviation from detrended historical data of given sensor/data type (force/torque)
double Classificator::getStd(Features type, int sensId)
{
    std::deque<double> detrended=m_meansHistory.at(type).at(sensId);   //get the historical data for given sensor/quantity
    for(int i=0;i<detrended.size();++i)
    {
        detrended.at(i)-=m_means.at(type).at(sensId); //detrend histroycal data
    }

    double detrendedMean=getMean(detrended);    //obatin new mean from data

    double squaredDiff=0.0;
    for(int i=0;i<detrended.size();++i)     //calculated nominator: sum of squared differences
    {
        squaredDiff+=pow((detrended.at(i)-detrendedMean),2);
    }

    double std=sqrt( ( squaredDiff/(detrended.size()-1) ) );    //standard deviation: squared root of nominator divided by samples num-1

    return std;

}

double Classificator::getMean(const std::deque<double>& data)
{

    double accumulator=0.0;
    for(int i=0;i<data.size();++i)
    {
        accumulator+=data.at(i);    //accumulate values
    }
    double mean=accumulator/data.size(); //calculate mean

    return mean;
}

//updates the means and the historical data used for mean/std calculation
void Classificator::updateMeans(const std::vector<double>& forceNorm, const std::vector<double>& torqueNorm)
{
    const std::vector<double>* data[]={&forceNorm, &torqueNorm};

    for(int ft=0;ft<FeaturesNum;++ft)       //for all the features (torque/force)
    {
        assert(data[ft]->size()==FINGERS_NUM);
        for(int sn=0;sn<FINGERS_NUM;++sn)   //for all the three sensors
        {
            if(m_meansHistory.at(ft).at(sn).size()<=NUM_INTIALISATION_VALS)    //if we don't have enough values
            {
                m_meansHistory.at(ft).at(sn).push_back(data[ft]->at(sn));    //blindly add value to history
            }
            else    //else refresh history with latest value and calculate the mean
            {
                m_calibrated=true;

                m_meansHistory.at(ft).at(sn).push_back(data[ft]->at(sn));    //add value
                m_meansHistory.at(ft).at(sn).pop_front();                   //circular buffer
                assert(m_meansHistory.at(ft).at(sn).size()==NUM_INTIALISATION_VALS);  //make sure history doesn't grow forever

                m_means.at(ft).at(sn)=getMean(m_meansHistory.at(ft).at(sn)); //obtain mean from current history
            }
        }
    }

}


//if this function return true, then the finger is close enough to an object to cast a decision
bool Classificator::isProximityTriggered(double tip, double pad)
{
    return (tip > PROX_TRIGGER) || (pad > PROX_TRIGGER);
}

//input: filtered force+proximity values, and filtered normalised torque percentage
const std::vector<Decision>& Classificator::decide(std::vector<double>& forceProx,std::vector<double>& torque)
{
    SensorName proxTipIds[]={ProximityTipFing1,ProximityTipFing1,ProximityTipFing1};
    SensorName proxPadIds[]={ProximityPadFing1,ProximityPadFing1,ProximityPadFing1};

    std::vector<double> forceNorm=Tactile::normaliseForce(forceProx);

    updateMeans(forceNorm,torque);  //update all the means of all the values and fingers

    for(int i=0;i<FINGERS_NUM && m_calibrated;++i)  //if the classifier hasn't booted, don't even bother checking
    {
        //if we are close to an object, we can cast a decision
        if( isProximityTriggered(forceProx.at(proxTipIds[i]),forceProx.at(proxPadIds[i])) )
        {

            m_classifications[i]=svm(getStd(FORCE, i) , getStd(TORQUE, i) );
        }
        else    //if too far, we have no clue
        {
            m_classifications[i]=CLASS_UNDECIDED;
        }
    }

    return m_classifications;
}

//-----------------------------------------------------------------------
//          UNIT TESTING
//-----------------------------------------------------------------------

#define TESTASSERT(val,check) if(val!=check)                                                              \
                              {                                                                           \
                                 std::cout << "> TEST ASSERTION FAILED: " << val << "!=" << check << std::endl;\
                                 assert(val==check);                                                      \
                               }


void Classificator::testMean()
{
    std::cout << "Testing meas" << std::endl;
    //test mean
    std::deque<double> testdata;
    for(int i=1;i<=5; ++i)
    {
        testdata.push_back(i);
    }

    double val=getMean(testdata);
    TESTASSERT(val,3);
    // test mean reverse
    testdata.clear();
    for(int i=5;i>0; --i)
    {
        testdata.push_back(i);
    }
    val=getMean(testdata);
    TESTASSERT(val,3);
    //test mean random
    testdata.clear();
    testdata.push_back(5);
    testdata.push_back(1);
    testdata.push_back(4);
    testdata.push_back(2);
    testdata.push_back(3);
    val=getMean(testdata);
    TESTASSERT(val,3);
    //------------------------------------
}

void Classificator::testConstructor()
{
    std::cout << "Testing costructor" << std::endl;
    //test means dims (no. of features)
    TESTASSERT(m_means.size(),FeaturesNum);
    //test inner means dim (no. of sensors)
    for(int i=0;i<FeaturesNum;++i)
    {
        TESTASSERT(m_means.at(i).size(),FINGERS_NUM);
    }
    //test mean values
    for(int i=0;i<FeaturesNum;++i)
    {
        for(int j=0;j<FINGERS_NUM;++j)
        {
            TESTASSERT(m_means.at(i).at(j),0.0);
        }
    }
    //test history dims (no. of features)
    TESTASSERT(m_meansHistory.size(),FeaturesNum);
    //test history dims (no. of sensors)
    for(int i=0;i<FeaturesNum;++i)
    {
        TESTASSERT(m_meansHistory.at(i).size(),FINGERS_NUM);
    }
    //test history content (empty set)
    for(int i=0;i<FeaturesNum;++i)
    {
        for(int j=0;j<FINGERS_NUM;++j)
        {
            TESTASSERT(m_meansHistory.at(i).at(j).size(),0);
        }
    }
}

void Classificator::cleanupHistory()
{

    for(int i=0;i<FeaturesNum;++i)
    {
        for(int j=0;j<FINGERS_NUM;++j)
        {
            int values=m_meansHistory.at(i).at(j).size();
            for(int k=0;k<values;++k)
            {
                m_meansHistory.at(i).at(j).pop_front(); //cleanup
            }
        }
    }
    testConstructor();  //make sure it is immaculate
}


void Classificator::testHistory()
{
    std::cout << "Testing history" << std::endl;
    //setup
    std::vector<double> forceTest;
    std::vector<double> torqueTest;
    for(int i=1;i<=FINGERS_NUM;++i)
    {
        forceTest.push_back(i/10);      //0.1 0.2 0.3
        torqueTest.push_back((4-i)/10); //0.3 0.2 0.1
    }

    //test single update
    updateMeans(forceTest, torqueTest);
    for(int i=0;i<FINGERS_NUM;++i)
    {
        TESTASSERT(m_meansHistory.at(FORCE).at(i).size(),1);
        TESTASSERT(m_meansHistory.at(FORCE).at(i).at(0),forceTest.at(i));
        TESTASSERT(m_meansHistory.at(TORQUE).at(i).size(),1);
        TESTASSERT(m_meansHistory.at(TORQUE).at(i).at(0),forceTest.at(i));
    }
    //test second update
    updateMeans(forceTest, torqueTest);
    for(int i=0;i<FINGERS_NUM;++i)
    {
        for(int j=0;j<2;++j)
        {
            TESTASSERT(m_meansHistory.at(FORCE).at(i).size(),1);
            TESTASSERT(m_meansHistory.at(FORCE).at(i).at(j),forceTest.at(i));
            TESTASSERT(m_meansHistory.at(TORQUE).at(i).size(),1);
            TESTASSERT(m_meansHistory.at(TORQUE).at(i).at(j),forceTest.at(i));
        }
    }
    forceTest.clear();
    torqueTest.clear();
    //test status
    TESTASSERT(m_calibrated,false);

    //reset (not ideal version)
    cleanupHistory();


    //test initialisation update (100 values)
    //setup
    for(int i=1;i<=100;++i)
    {
        for(int j=0;j<FINGERS_NUM;++j)
        {
            forceTest.push_back(i/101);      //each simulated reading is a triplet of identical values, from 0.01 to ~1.0
            torqueTest.push_back(i/101);
        }
        updateMeans(forceTest, torqueTest);
        forceTest.clear();
        torqueTest.clear();
    }
    //we need 101 values to kick off
    TESTASSERT(m_calibrated,false);

    //test kick off - setup
    for(int j=0;j<FINGERS_NUM;++j)
    {
        forceTest.push_back(1);
        torqueTest.push_back(1);
    }
    updateMeans(forceTest, torqueTest);
    forceTest.clear();
    torqueTest.clear();
    //test kick off
    TESTASSERT(m_calibrated,true);
    //test means
    for(int j=0;j<FINGERS_NUM;++j)
    {
        TESTASSERT(m_means.at(FORCE).at(j),0.505);
        TESTASSERT(m_means.at(TORQUE).at(j),0.505);
    }

    //test running for some time doesn't altern means and system
    //setup
    for(int i=1;i<=100;++i)
    {
        for(int j=0;j<FINGERS_NUM;++j)
        {
            forceTest.push_back(i/101);      //each simulated reading is a triplet of identical values, from 0.01 to ~1.0
            torqueTest.push_back(i/101);
        }
        updateMeans(forceTest, torqueTest);
        forceTest.clear();
        torqueTest.clear();
    }
    //test
    for(int j=0;j<FINGERS_NUM;++j)  //same amount of values x2 shall not alter the mean
    {
        TESTASSERT(m_means.at(FORCE).at(j),0.505);
        TESTASSERT(m_means.at(TORQUE).at(j),0.505);
    }

    //cleanup
    cleanupHistory();
}

//this function is to perform local unit testing
void Classificator::autotest()
{
    std::cout << "Autotesting Classificator..." << std::endl;
    //test constructor
    testConstructor();
    std::cout << "testConstructor passed" << std::endl;
    //mean function
    testMean();
    std::cout << "testMean passed" << std::endl;
    //history
    testHistory();
    std::cout << "testHistory passed" << std::endl;
}



