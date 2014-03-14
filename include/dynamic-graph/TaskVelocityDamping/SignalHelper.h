#ifndef SIGNALHELPER_H
#define SIGNALHELPER_H

#include <boost/shared_ptr.hpp>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

namespace dynamicgraph{

namespace SignalHelper{

typedef SignalTimeDependent < sot::MatrixHomogeneous, int > SignalTimeMatrix;
typedef SignalTimeDependent < dynamicgraph::Vector, int > SignalTimeVector;
typedef SignalTimeDependent < double, int > SignalTimeDouble;

typedef SignalPtr<dynamicgraph::Matrix, int > SignalPtrMatrix;
typedef SignalPtr<dynamicgraph::Vector, int > SignalPtrVector;
typedef SignalPtr<double, int > SignalPtrDouble;


/*TEMPLATING HERE*/


inline boost::shared_ptr<SignalTimeVector> createOutputSignalTimeVector(std::string name){

    std::string signal_name = "TaskVelocityDamping(taskDynamicVelocityvelDamp)::output(vector)::"+name;
    boost::shared_ptr<SignalTimeVector> signal =  boost::shared_ptr<SignalTimeVector>(
                new SignalTimeVector(NULL, signal_name));
    return signal;
}

inline boost::shared_ptr<SignalTimeDouble> createOutputSignalTimeDouble(std::string name){

    std::string signal_name = "TaskVelocityDamping(taskDynamicVelocityvelDamp)::output(double)::"+name;
    boost::shared_ptr<SignalTimeDouble> signal =  boost::shared_ptr<SignalTimeDouble>(
                new SignalTimeDouble(NULL, signal_name));
    return signal;
}

inline boost::shared_ptr<SignalPtrMatrix> createInputSignalMatrix(std::string name){
    std::string signal_name = "TaskVelocityDamping(taskDynamicVelocityvelDamp)::input(matrix)::"+name;
    boost::shared_ptr<SignalPtrMatrix> signal = boost::shared_ptr<SignalPtrMatrix> (
                new SignalPtrMatrix(NULL, signal_name));
    return signal;
}

inline boost::shared_ptr<SignalPtrVector> createInputSignalVector(std::string name){
    std::string signal_name = "TaskVelocityDamping(taskDynamicVelocityvelDamp)::input(vector)::"+name;
    boost::shared_ptr<SignalPtrVector> signal = boost::shared_ptr<SignalPtrVector> (
                new SignalPtrVector(NULL, signal_name));
    return signal;
}

inline boost::shared_ptr<SignalPtrDouble> createInputSignalDouble(std::string name){
    std::string signal_name = "TaskVelocityDamping(taskDynamicVelocityvelDamp)::input(double)::"+name;
    boost::shared_ptr<SignalPtrDouble> signal = boost::shared_ptr<SignalPtrDouble> (
                new SignalPtrDouble(NULL, signal_name));
    return signal;
}

}
}
#endif
