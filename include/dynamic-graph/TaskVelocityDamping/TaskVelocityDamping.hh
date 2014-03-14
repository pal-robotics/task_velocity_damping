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

//#include <sot_transform/sot_frame_transform.h>
#include <tf/transform_broadcaster.h>

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

    DECLARE_SIGNAL_IN(dt,double);
    DECLARE_SIGNAL_IN(controlGain,double);

public:  /* --- COMPUTATION --- */

    dg::sot::VectorMultiBound& computeTask( dg::sot::VectorMultiBound& res,int time );
    ml::Matrix& computeJacobian( ml::Matrix& J,int time );

    void setAvoidingObjects(const std::string& avoiding_objects);
    void setAvoidingObjectPair(const std::string& p1, const std::string& p2);
    void finalizeSignals();

private:

//    boost::shared_ptr<SotFrameTransformer> sot_transformer_;

    // input signals for P1
    std::vector<boost::shared_ptr< SignalPtr <dynamicgraph::Matrix, int> > > p1_vec;
    // input signals for p2
    std::vector<boost::shared_ptr< SignalPtr <dynamicgraph::Matrix, int> > > p2_vec;
    // input signals for jacobians of p1
    std::vector<boost::shared_ptr< SignalPtr <dynamicgraph::Matrix, int> > > jVel_vec;
    // input signals for influence distance
    std::vector<boost::shared_ptr< SignalPtr <double, int> > > di_vec;
    // input signals for influence distance
    std::vector<boost::shared_ptr< SignalPtr <double, int> > > ds_vec;

    // output signals for n and distance
    std::vector<boost::shared_ptr< SignalTimeDependent <dynamicgraph::Vector, int> > > n_vec;
    std::vector<boost::shared_ptr< SignalTimeDependent <dynamicgraph::Vector, int> > > v_vec;
    std::vector<boost::shared_ptr< SignalTimeDependent <double, int> > > d_vec;

    void split(std::vector<std::string> &tokens, const std::string &text, char sep) const;


    double calculateDistance(sot::MatrixHomogeneous p1, sot::MatrixHomogeneous p2);
    ml::Vector calculateDirectionalVector(sot::MatrixHomogeneous p1,sot::MatrixHomogeneous p2);
    ml::Vector calculateUnitVector(sot::MatrixHomogeneous p1,sot::MatrixHomogeneous p2);

    int avoidance_size_;
    std::vector<std::string> avoidance_objects_;

    std::vector<std::string> collision_pair_names_;

    tf::TransformBroadcaster br_;

}; // class TaskVelocityDamping

inline tf::Transform transformToTF(const dynamicgraph::Vector& vector)
{
    tf::Transform transform;
    transform.setIdentity();

    tf::Vector3 pos;
    pos.setValue(vector.elementAt(0),vector.elementAt(1),vector.elementAt(2));
    transform.setOrigin(pos);
    return transform;
}

inline tf::Transform transformToTF(const dynamicgraph::Matrix& matrix){
    tf::Transform transform;

    tf::Matrix3x3 rot;
    rot.setValue(
                matrix.elementAt(0,0),
                matrix.elementAt(0,1),
                matrix.elementAt(0,2),

                matrix.elementAt(1,0),
                matrix.elementAt(1,1),
                matrix.elementAt(1,2),

                matrix.elementAt(2,0),
                matrix.elementAt(2,1),
                matrix.elementAt(2,2)

    );

    tf::Vector3 pos;
    pos.setValue(matrix.elementAt(0,3),matrix.elementAt(1,3),matrix.elementAt(2,3));

    tf::Quaternion quat;
    rot.getRotation(quat);


    // check that!!
    transform.setRotation(quat);
    transform.setOrigin(pos);

    return transform;
}

} // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __sot_TaskVelocityDamping_H__

