#ifndef CLASSIFICATOR_H
#define CLASSIFICATOR_H

#include <vector>
#include <deque>
#include "common_defines.h"


enum Features
{
    FORCE,
    TORQUE,
    FeaturesNum
};

///Implementation of SVM classificator for distinguishing hard from soft objects
class Classificator
{

    //this object cannot be copied
    Classificator& operator=(Classificator&);
    Classificator(const Classificator&);

    static const double FORCE_PAR;  //a F +
    static const double TORQUE_PAR; //b T +
    static const double FREE_PAR;   //c

    //if a value falls withing this threashold (+/-) it is ambiguos
    static const double AMBIGUITY_THREAS;
    static const double PROX_TRIGGER;

    volatile bool m_calibrated;  //true if every internal parameter is ready
    std::vector<Decision> m_classifications;    //classification results
    std::vector< std::vector< std::deque<double> > > m_meansHistory;    //values of torque/foirce for mean calculation => {vector1:[force|torque]{vector2:[sensor1|2|3]{vector3:[value1|2....N]}}}
    std::vector<std::vector<double> > m_means;  //overall means => {vector1:[force|torque]vector2:[mean of sensor1|2|3]}

    //classifies force/torque of a single finger using linear svm (f=force, t=torque)
    Decision svm(double stdf, double stdt);
    //if this function return true a classification takes place, otherwise we return undetected
    bool isProximityTriggered(double tip, double pad);
    //update the internal record of means and hisotrical data
    void updateMeans(const std::vector<double>& forceNorm, const std::vector<double>& torqueNorm);
    //calculate standard deviation from internal data
    double getStd(Features type, int sensorId);
    //calculate mean from N values
    double getMean(const std::deque<double>& data);

    //internal unit tests
    void testMean();
    void testConstructor();
    void testHistory();
    void cleanupHistory();
public:

    Classificator();
    ~Classificator(){}

    //main entry point: receives force/torque/proximity data and outputs a decision for all fingers
    const std::vector<Decision>& decide(std::vector<double>& forceProx,std::vector<double>& torque);

    //triggers auto-unit testing
    void autotest();
};


#endif // CLASSIFICATOR_H
