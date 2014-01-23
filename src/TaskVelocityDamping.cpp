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
#include <dynamic-graph/TaskVelocityDamping/TaskVelocityDamping.hh>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph
{
namespace sot
{

namespace dg = ::dynamicgraph;


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
    ,CONSTRUCT_SIGNAL_IN(p1,sot::MatrixHomogeneous)
//    p2 is considered as being fixed/static
    ,CONSTRUCT_SIGNAL_IN(p2,sot::MatrixHomogeneous)
    ,CONSTRUCT_SIGNAL_IN(jVel,ml::Matrix)
    ,CONSTRUCT_SIGNAL_IN(dt,double)
    ,CONSTRUCT_SIGNAL_IN(controlGain,double)
    ,CONSTRUCT_SIGNAL_IN(selec,Flags)
    ,CONSTRUCT_SIGNAL_IN(di, double)
    ,CONSTRUCT_SIGNAL_IN(ds, double)
    ,distanceSOUT("TaskVelocityDamping("+name+")::output(double)::distance")
    ,nSOUT("TaskVelocityDamping("+name+")::output(ml::Vector)::n")
    ,vSOUT("TaskVelocityDamping("+name+")::output(ml::Vector)::v")


{
    taskSOUT.setFunction( boost::bind(&TaskVelocityDamping::computeTask,this,_1,_2) );
    jacobianSOUT.setFunction( boost::bind(&TaskVelocityDamping::computeJacobian,this,_1,_2) );
    distanceSOUT.setFunction(boost::bind(&TaskVelocityDamping::computeDistance, this, _1, _2));
    nSOUT.setFunction(boost::bind(&TaskVelocityDamping::computeN, this, _1, _2));
    vSOUT.setFunction(boost::bind(&TaskVelocityDamping::computeV, this, _1, _2));

    taskSOUT.clearDependencies();
    taskSOUT.addDependency(p1SIN );
    taskSOUT.addDependency(p2SIN);
    taskSOUT.addDependency(distanceSOUT);

    taskSOUT.addDependency( dtSIN );
    taskSOUT.addDependency(dsSIN);
    taskSOUT.addDependency(diSIN);
    taskSOUT.addDependency(controlGainSIN);

    jacobianSOUT.clearDependencies();
    jacobianSOUT.addDependency(p1SIN);
    jacobianSOUT.addDependency(p2SIN);
    jacobianSOUT.addDependency(nSOUT);

    distanceSOUT.clearDependencies();
    distanceSOUT.addDependency(p1SIN);
    distanceSOUT.addDependency(p2SIN);

    nSOUT.clearDependencies();
    nSOUT.addDependency(p1SIN);
    nSOUT.addDependency(p2SIN);

    vSOUT.clearDependencies();
    vSOUT.addDependency(p1SIN);
    vSOUT.addDependency(p2SIN);

    controlGainSIN = 1.0;

    // Commands
    std::string docstring;

    // setVelocity
    docstring =
            "\n"
            " Insert a distance vector for damping the velocity in proportion to distance."
            "\n";

    signalRegistration(p1SIN << p2SIN << jVelSIN << dtSIN << controlGainSIN << diSIN << dsSIN << distanceSOUT << nSOUT << vSOUT);

}
double& TaskVelocityDamping::computeDistance(double& res, int time)
{
    const sot::MatrixHomogeneous& p1 = p1SIN(time);
    const sot::MatrixHomogeneous& p2 = p2SIN(time);
    res = calculateDistance(p1, p2);
    distanceSOUT.setConstant(res);
    return res;
}

ml::Vector& TaskVelocityDamping::computeN(ml::Vector& res, int time)
{
    const sot::MatrixHomogeneous & p1 = p1SIN(time);
    const sot::MatrixHomogeneous & p2 = p2SIN(time);
    res = calculateUnitVector(p1, p2);
    nSOUT.setConstant(res);
    return res;
}

ml::Vector& TaskVelocityDamping::computeV(ml::Vector& res, int time)
{
    const sot::MatrixHomogeneous & p1 = p1SIN(time);
    const sot::MatrixHomogeneous & p2 = p2SIN(time);
    res = calculateDirectionalVector(p1, p2);
    vSOUT.setConstant(res);
    return res;
}

/* ---------------------------------------------------------------------- */
/* --- COMPUTATION ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

ml::Vector TaskVelocityDamping::calculateDirectionalVector(sot::MatrixHomogeneous p1, sot::MatrixHomogeneous p2){
    sot::MatrixHomogeneous diff;
    diff = p1.substraction(p2);
    ml::Vector dist_vec(3);
    return diff.extract(dist_vec);
}

ml::Vector TaskVelocityDamping::calculateUnitVector(sot::MatrixHomogeneous p1, sot::MatrixHomogeneous p2){

    ml::Vector n(3);
    double d = calculateDistance(p1, p2);
    n = calculateDirectionalVector(p1, p2);
    n = n.multiply(float(1/d));
    // check for correct computation of normalization
    assert(fabs(n.norm()-1.0) < 0.01);

    return n;
}

double TaskVelocityDamping::calculateDistance(sot::MatrixHomogeneous p1, sot::MatrixHomogeneous p2){
    ml::Vector dist_vec = calculateDirectionalVector(p1, p2);
    double distnorm = dist_vec.norm();
    return distnorm;
}

dg::sot::VectorMultiBound& TaskVelocityDamping::computeTask( dg::sot::VectorMultiBound& res,int time )
{
    const double& ds = dsSIN(time);
    const double& di = diSIN(time);
//    const double& dt = dtSIN(time);
    double epsilon = controlGainSIN(time);

    MultiBound::SupInfType bound = MultiBound::BOUND_INF;
    double d;
    computeDistance(d, time);

    distanceSOUT.setConstant(d);
    double upperFrac =  d-ds;
    double lowerFrac =  (di-ds);
    double fraction = - epsilon *(upperFrac / lowerFrac);

    res.resize(1);
    res[0] = dg::sot::MultiBound(fraction, bound);

    return res;
}

ml::Matrix& TaskVelocityDamping::
computeJacobian( ml::Matrix& J,int time )
{
    const ml::Matrix& jacobian = jVelSIN(time);

    /* original paper implementation */

    ml::Vector dist_v;
    computeV(dist_v, time);

    ml::Vector mat_n;
    computeN(mat_n, time);

    ml::Matrix mat_n_transpose(1,dist_v.size());
    for (unsigned int i=0; i<mat_n.size(); ++i){
        mat_n_transpose(0,i) = mat_n(i);
    }
    // extract only position information
    ml::Matrix jacobianPos;
    jacobian.extract(0,0,3,jacobian.nbCols(), jacobianPos);

    J=mat_n_transpose.multiply(jacobianPos);
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

