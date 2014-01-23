/*
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

#ifndef __sot_TaskVelocityDamping_H__
#define __sot_TaskVelocityDamping_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <sot/core/task.hh>
#include <sot/core/flags.hh>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/multi-bound.hh>

#include <iostream>

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class TaskVelocityDamping
        :public TaskAbstract
        ,public EntityHelper<TaskVelocityDamping>
{

public: /* --- CONSTRUCTOR ---- */

    TaskVelocityDamping( const std::string& name );

public: /* --- ENTITY INHERITANCE --- */

    DYNAMIC_GRAPH_ENTITY_DECL();
    virtual void display( std::ostream& os ) const;

public:  /* --- SIGNALS --- */

    DECLARE_SIGNAL_IN(p1,sot::MatrixHomogeneous);
    DECLARE_SIGNAL_IN(p2,sot::MatrixHomogeneous);
    DECLARE_SIGNAL_IN(jVel,ml::Matrix);
    DECLARE_SIGNAL_IN(dt,double);
    DECLARE_SIGNAL_IN(controlGain,double);
    DECLARE_SIGNAL_IN(selec,Flags);
    DECLARE_SIGNAL_IN(di,double);
    DECLARE_SIGNAL_IN(ds,double);
    DECLARE_SIGNAL_OUT(distance, double);
    DECLARE_SIGNAL_OUT(n,ml::Vector);
    DECLARE_SIGNAL_OUT(v,ml::Vector);
//    DECLARE_SIGNAL_OUT(activeSize,int);

public:  /* --- COMPUTATION --- */
    dg::sot::VectorMultiBound& computeTask( dg::sot::VectorMultiBound& res,int time );
    ml::Matrix& computeJacobian( ml::Matrix& J,int time );
    double& computeDistance(double& res, int time);
    ml::Vector& computeN(ml::Vector& res, int time);
    ml::Vector& computeV(ml::Vector& res, int time);
//    void setDistanceVector(const ml::Vector& distanceIn);
private:
//    double di;
//    double distance;
//    double ds;
    double calculateDistance(sot::MatrixHomogeneous p1, sot::MatrixHomogeneous p2);
    ml::Vector calculateDirectionalVector(sot::MatrixHomogeneous p1,sot::MatrixHomogeneous p2);
    ml::Vector calculateUnitVector(sot::MatrixHomogeneous p1,sot::MatrixHomogeneous p2);

}; // class TaskVelocityDamping

} // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __sot_TaskVelocityDamping_H__

