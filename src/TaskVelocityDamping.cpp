/*
 * Copyright (c) 2013, PAL Robotics, S.L.
 * Copyright 2011, Nicolas Mansard, LAAS-CNRS
 *
 * This file is part of sot-dyninv.
 * sot-dyninv is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-dyninv is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-dyninv.  If not, see <http://www.gnu.org/licenses/>.
 */

/** \author Karsten.Knese at googlemail.com
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

//#define VP_DEBUG
#define VP_DEBUG_MODE 15
#include <sot/core/debug.hh>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <dynamic-graph/TaskVelocityDamping/TaskVelocityDamping.hh>
#include <dynamic-graph/TaskVelocityDamping/SignalHelper.h>

#include <tf/transform_datatypes.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph
{
namespace sot
{

class SetCollisionPair : public dynamicgraph::command::Command {

public:
    virtual ~SetCollisionPair () {}

    SetCollisionPair(TaskVelocityDamping& entity, const std::string& docstring) :
        Command(entity, boost::assign::list_of(command::Value::STRING)
                (command::Value::STRING), docstring) {}

    command::Value doExecute()
    {
        TaskVelocityDamping& task = static_cast<TaskVelocityDamping&>(owner());
        std::vector<command::Value> values = getParameterValues();
        std::string p1 = values[0].value();
        std::string p2 = values[1].value();
        task.setAvoidingObjectPair(p1,p2);
        return command::Value ();
    }
}; // class SetCollisionPair

namespace dg = ::dynamicgraph;

typedef SignalTimeDependent < sot::MatrixHomogeneous, int > SignalTimeMatrix;
typedef SignalTimeDependent < dynamicgraph::Vector, int > SignalTimeVector;
typedef SignalTimeDependent < double, int > SignalTimeDouble;
typedef SignalPtr<dynamicgraph::Matrix, int > SignalPtrMatrix;
typedef SignalPtr<dynamicgraph::Vector, int > SignalPtrVector;
typedef SignalPtr<double, int > SignalPtrDouble;

/* --- DG FACTORY ------------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskVelocityDamping,"TaskVelocityDamping");

/* ---------------------------------------------------------------------- */
/* --- CONSTRUCTION ----------------------------------------------------- */
/* ---------------------------------------------------------------------- */

/*
  Implementation according to paper reference:
A Local Collision Avoidance Method
for Non-strictly Convex Polyhedra

*/

TaskVelocityDamping::TaskVelocityDamping( const std::string & name )
    : TaskAbstract(name)

    //    p1 is considered as the moving point
    //    p2 is considered as being fixed/static
    ,CONSTRUCT_SIGNAL_IN(dt,double)
    ,CONSTRUCT_SIGNAL_IN(controlGain,double)
    //    , sot_transformer_(new SotFrameTransformer())
    , br_()

{

    // initialize which collision pairs should be on the same hierarchy level
    std::string docstring;
    docstring =
            "\n"
            "    Initializes collisions for task avoidance\n"
            "      takes a string of signal names (separated by :) for collision pairs \n"
            "     and creates a signal input socket (p1,p2, jVel_p1) for each collision object"
            "\n";
    addCommand(std::string("setAvoidingObjectString"),
               new dynamicgraph::command::Setter<TaskVelocityDamping, std::string >
               (*this, &TaskVelocityDamping::setAvoidingObjects, docstring));

    docstring =
            "\n"
            "    Initializes collisions for task avoidance\n"
            "      takes two string for p1 and p2 \n"
            "\n";
    addCommand(std::string("setAvoidingObjectPair"),new SetCollisionPair(*this, docstring)
               );

    docstring =
            "\n"
            "    Initializes collisions for fcl\n"
            "      comes in combination with add_collision_joint  \n"
            "    when all joints are added, trigger to function to setup the signalptrs"
            "\n";
    addCommand(std::string("finalizeSignals"),
               dynamicgraph::command::makeCommandVoid0(*this, &TaskVelocityDamping::finalizeSignals, docstring));

    taskSOUT.setFunction( boost::bind(&TaskVelocityDamping::computeTask,this,_1,_2) );
    jacobianSOUT.setFunction( boost::bind(&TaskVelocityDamping::computeJacobian,this,_1,_2) );

    controlGainSIN = 1.0;

    signalRegistration( dtSIN << controlGainSIN );

}

void TaskVelocityDamping::split(std::vector<std::string> &tokens, const std::string &text, char sep) const {
    int start = 0, end = 0;
    while ((end = text.find(sep, start)) != std::string::npos) {
        tokens.push_back(text.substr(start, end - start));
        start = end + 1;
    }
    tokens.push_back(text.substr(start));
}

void TaskVelocityDamping::setAvoidingObjectPair(const std::string& p1, const std::string& p2){
    // outgoing signal name will always be moving part (P1) towards static part (P2)
    // ex:  arm_right_7_joint against torso_2_joint => signalname: arm_right_7_jointtorso_2_joint
    collision_pair_names_.push_back((p1+p2));
}

void TaskVelocityDamping::finalizeSignals()
{
    avoidance_size_  = collision_pair_names_.size();

    //    sot_transformer_->initVectorSize(avoidance_size_);
    p1_vec.resize(avoidance_size_);
    p2_vec.resize(avoidance_size_);
    jVel_vec.resize(avoidance_size_);
    n_vec.resize(avoidance_size_);
    v_vec.resize(avoidance_size_);
    d_vec.resize(avoidance_size_);
    ds_vec.resize(avoidance_size_);
    di_vec.resize(avoidance_size_);

    for (int idx = 0; idx < avoidance_size_; ++idx) {

        //        sot_transformer_->setSOTFrameTransform(avoidance_objects[var], var);

        boost::shared_ptr<SignalPtrMatrix> p1_signal  = SignalHelper::createInputSignalMatrix("p1_"+collision_pair_names_[idx]);
        signalRegistration(*p1_signal );
        p1_vec[idx] = p1_signal;
        std::cerr << "registered p1 signal: p1_"<<collision_pair_names_[idx] <<std::endl;

        boost::shared_ptr<SignalPtrMatrix> p2_signal  = SignalHelper::createInputSignalMatrix("p2_"+collision_pair_names_[idx]);
        signalRegistration(*p2_signal );
        p2_vec[idx] = p2_signal;
        std::cerr << "registered p1 signal: p2_"<<collision_pair_names_[idx] <<std::endl;

        boost::shared_ptr<SignalPtrMatrix> jVel_signal  = SignalHelper::createInputSignalMatrix("jVel_"+collision_pair_names_[idx]);
        signalRegistration(*jVel_signal );
        jVel_vec[idx] = jVel_signal;
        std::cerr << "registered p1 signal: jVel_"<<collision_pair_names_[idx] <<std::endl;

        boost::shared_ptr<SignalPtrDouble> ds_signal  = SignalHelper::createInputSignalDouble("ds_"+collision_pair_names_[idx]);
        signalRegistration(*ds_signal );
        ds_vec[idx] = ds_signal;
        std::cerr << "registered ds signal: ds_"<<collision_pair_names_[idx] <<std::endl;

        boost::shared_ptr<SignalPtrDouble> di_signal  = SignalHelper::createInputSignalDouble("di_"+collision_pair_names_[idx]);
        signalRegistration(*di_signal );
        di_vec[idx] = di_signal;
        std::cerr << "registered di signal: di_"<<collision_pair_names_[idx] <<std::endl;

        boost::shared_ptr<SignalTimeVector> n_signal = SignalHelper::createOutputSignalTimeVector("n_"+collision_pair_names_[idx]);
        signalRegistration(*n_signal);
        n_vec[idx] = n_signal;

        boost::shared_ptr<SignalTimeVector> v_signal = SignalHelper::createOutputSignalTimeVector("v_"+collision_pair_names_[idx]);
        signalRegistration(*v_signal);
        v_vec[idx] = v_signal;

        boost::shared_ptr<SignalTimeDouble> d_signal = SignalHelper::createOutputSignalTimeDouble("d_"+collision_pair_names_[idx]);
        signalRegistration(*d_signal);
        d_vec[idx] = d_signal;

        taskSOUT.addDependency(*p1_signal);
        taskSOUT.addDependency(*p2_signal);

        jacobianSOUT.addDependency(*p1_signal);
        jacobianSOUT.addDependency(*p2_signal);
        jacobianSOUT.addDependency(*jVel_signal);

    }
}


void TaskVelocityDamping::setAvoidingObjects(const std::string& avoiding_objects_string)
{
    std::cerr << "received avoiding objects: " << avoiding_objects_string <<std::endl;
    std::vector<std::string> avoidance_objects;
    split(avoidance_objects, avoiding_objects_string, ':');
    std::cerr << "will register objects: " << avoidance_objects.size();

    collision_pair_names_ = avoidance_objects;

    finalizeSignals();
}

/* ---------------------------------------------------------------------- */
/* --- COMPUTATION ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

ml::Vector TaskVelocityDamping::calculateDirectionalVector(sot::MatrixHomogeneous p1, sot::MatrixHomogeneous p2)
{
    sot::MatrixHomogeneous diff;
    diff = p1.substraction(p2);
    ml::Vector dist_vec(3);
    return diff.extract(dist_vec);
}

ml::Vector TaskVelocityDamping::calculateUnitVector(sot::MatrixHomogeneous p1, sot::MatrixHomogeneous p2)
{
    ml::Vector n(3);
    double d = calculateDistance(p1, p2);
    n = calculateDirectionalVector(p1, p2);
    n = n.multiply(float(1/d));
    // check for correct computation of normalization
    if(fabs(n.norm()-1.0) > 0.01) // the norm should be in the tolerance
    {
        std::cerr << "difference" << fabs(n.norm()-1.0) << std::endl;
        std::cerr << "something fishy with n-vector "<< n << std::endl;
    }
    return n;
}

double TaskVelocityDamping::calculateDistance(sot::MatrixHomogeneous p1, sot::MatrixHomogeneous p2)
{
    ml::Vector dist_vec = calculateDirectionalVector(p1, p2);
    double distnorm = dist_vec.norm();
    return distnorm;
}

dg::sot::VectorMultiBound& TaskVelocityDamping::computeTask( dg::sot::VectorMultiBound& res,int time )
{
    // first time initialization or reaction if more objects are coming
    if (res.size() != avoidance_size_){
        res.resize(avoidance_size_);
    }
    //    const double& dt = dtSIN(time);
    double epsilon = controlGainSIN(time);

    for (int idx = 0; idx < avoidance_size_; ++idx) {

        const sot::MatrixHomogeneous& p1 = (*p1_vec[idx])(time);
        const sot::MatrixHomogeneous& p2 = (*p2_vec[idx])(time);

        const double& ds = (*ds_vec[idx])(time);
        const double& di = (*di_vec[idx])(time);

        MultiBound::SupInfType bound = MultiBound::BOUND_INF;
        double d = calculateDistance(p1, p2);
        double upperFrac =  d-ds;
        double lowerFrac =  di-ds;
        double fraction = - epsilon *(upperFrac / lowerFrac);

        d_vec[idx]->setConstant(d);
        res[idx] = dg::sot::MultiBound(fraction, bound);
    }
    return res;
}

ml::Matrix& TaskVelocityDamping::
computeJacobian( ml::Matrix& J,int time )
{

    // output Jacobian has <avoidance_size> x < jacobian.nbCols()>
    const int col_count = (*jVel_vec[0])(time).nbCols();
    if (J.nbCols() != col_count ){
        J.resize(avoidance_size_,col_count);
    }
    if (J.nbRows() != avoidance_size_){
        J.resize(avoidance_size_,col_count);
    }

    //    std::cerr << "jacobian dimension: " << J.nbRows() << " x " << J.nbCols() << std::endl;
    // use first jVel signal to resize matrix
    // then matrix can easily be stacked into each other
    // cannot use mal:: stackMatrix because it's not implemented
    for (int idx = 0; idx < avoidance_size_; ++idx) {

        const sot::MatrixHomogeneous& p1 = (*p1_vec[idx])(time);
        const sot::MatrixHomogeneous& p2 = (*p2_vec[idx])(time);

        // adjust coordinates for correct directional unit vector computation!
        // critical

        //        br_.sendTransform(
        //                    tf::StampedTransform(transformToTF(p1), ros::Time::now(), "base_link",
        //                                         "taskDAMP_p1in_"+avoidance_objects_vec[var]));
        //        br_.sendTransform(
        //                    tf::StampedTransform(transformToTF(p2), ros::Time::now(), "base_link",
        //                                         "taskDAMP_p2in_"+avoidance_objects_vec[var]));


        const ml::Matrix& jacobian = (*jVel_vec[idx])(time);

        ml::Vector mat_n = calculateUnitVector(p1,p2);
        ml::Vector v = calculateDirectionalVector(p1, p2);


        // CHECK VIA TF THAT THIS MAT_N IS COMPUTED CORRECTLY INSIDE THE SOT CALCULATION
        // BRUTAL HACK !!!!!
        size_t rightpos = collision_pair_names_[idx].find("right");
        size_t leftpos = collision_pair_names_[idx].find("left");
        size_t endeffector = collision_pair_names_[idx].find("tool");

        // exclude endeffector as this is not getting parsed by jrl
        if (endeffector == std::string::npos){

            // if (right and no left) or (right and left and right < left)
            if ( (rightpos != std::string::npos && leftpos == std::string::npos)
                 ||
                 (rightpos != std::string::npos && leftpos != std::string::npos && rightpos < leftpos) )
            {
                v.elementAt(1) = v.elementAt(1)*-1;
                v.elementAt(2) = v.elementAt(2)*-1;

                mat_n.elementAt(1) = mat_n.elementAt(1)*(-1);
                mat_n.elementAt(2) = mat_n.elementAt(2)*(-1);
            }
            else{
                v.elementAt(0) = v.elementAt(0)*-1;
                mat_n.elementAt(0) = mat_n.elementAt(0)*(-1);
            }
        }
        //        br_.sendTransform(
        //                    tf::StampedTransform(transformToTF(v), ros::Time::now(), "taskDAMP_p2in_"+avoidance_objects_vec[var],
        //                                         "taskDAMP_v_"+avoidance_objects_vec[var]));

        v_vec[idx]->setConstant(v);
        n_vec[idx]->setConstant(mat_n);

        ml::Matrix mat_n_transpose(1,mat_n.size());
        for (unsigned int i=0; i<mat_n.size(); ++i){
            mat_n_transpose(0,i) = mat_n(i);
        }

        // extract only position information
        ml::Matrix jacobianPos;
        jacobian.extract(0,0,3,jacobian.nbCols(), jacobianPos);

        ml::Matrix partJacobian;
        partJacobian = mat_n_transpose.multiply(jacobianPos);

        for (int jIdx = 0; jIdx < col_count; ++jIdx) {
            J.elementAt(idx, jIdx) = partJacobian.elementAt(0,jIdx);
        }

    }
    return J;

}

/* ------------------------------------------------------------------------ */
/* --- DISPLAY ENTITY ----------------------------------------------------- */
/* ------------------------------------------------------------------------ */

void TaskVelocityDamping::
display( std::ostream& os ) const
{
    os << "TaskVelocityDamping " << name << ": " << std::endl;
}

} // namespace sot
} // namespace dynamicgraph

