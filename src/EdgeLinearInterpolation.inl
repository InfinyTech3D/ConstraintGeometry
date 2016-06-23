#ifndef SOFA_COMPONENT_EDGELINEARINTERPOLATION_INL
#define SOFA_COMPONENT_EDGELINEARINTERPOLATION_INL

#include "EdgeLinearInterpolation.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaOpenglVisual/OglModel.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaConstraint/BilateralInteractionConstraint.h>
#include <SofaBaseMechanics/MechanicalObject.h>

namespace sofa {

namespace core {

namespace behavior {

template<class DataTypes>
void EdgeLinearInterpolation<DataTypes>::fillProximity(const Coord & P,ConstraintProximity & pinfo) {
    const helper::ReadAccessor<Data <VecCoord> >& x = *this->m_state->read(core::VecCoordId::position());

    double minDist = 0;
    for(int e=0;e<this->m_container->getNbEdges();e++) {
        double fact_u;
        double fact_v;

        topology::Edge edge = this->m_container->getEdge(e);

        Coord v = x[edge[1]] - x[edge[0]];
        fact_v = dot (P - x[edge[0]],v) / dot (v,v);

        if (fact_v<0.0) fact_v = 0.0;
        else if (fact_v>1.0) fact_v = 1.0;

        fact_u = 1.0-fact_v;

        Coord Q = x[edge[0]] * fact_u + x[edge[1]] * fact_v;

        //1.0 + 0.1 i.e. 0.1 is to avoid zero
        double dist = (Q-P).norm();

        if ((e==0) || (dist < minDist)) {
            pinfo.pid.resize(2);
            pinfo.fact.resize(2);

            pinfo.pid[0] = edge[0];
            pinfo.pid[1] = edge[1];

            pinfo.fact[0] = fact_u;
            pinfo.fact[1] = fact_v;

            minDist = dist;
        }
    }
}

template<class DataTypes>
void EdgeLinearInterpolation<DataTypes>::fillProximity(unsigned pid,ConstraintProximity & pinfo) {
    pinfo.pid.resize(1);
    pinfo.fact.resize(1);

    pinfo.pid[0] = pid;
    pinfo.fact[0] = 1.0;
}

template<class DataTypes>
void EdgeLinearInterpolation<DataTypes>::fillConstraintNormal(const ConstraintProximity & pinfo,ConstraintNormal & ninfo) {
    const helper::ReadAccessor<Data <VecCoord> >& x = *this->m_state->read(core::VecCoordId::position());

    //Compute the normals
    Vector3 N1;

    if (pinfo.pid.size()==1) {
        if (pinfo.pid[0]==x.size()-1) N1 = x[pinfo.pid[0]] - x[pinfo.pid[0]-1];
        else N1 = x[pinfo.pid[0]+1] - x[pinfo.pid[0]];
    } else {
        N1 = x[pinfo.pid[1]] - x[pinfo.pid[0]];
    }

    N1.normalize();

    Vector3 N2 = cross(N1,((fabs(dot(N1,Vector3(1,0,0)))>0.999999) ? Vector3(0,1,0) : Vector3(1,0,0)));
    N2.normalize();

    Vector3 N3 = cross(N1,N2);
    N3.normalize();

    ninfo.normals.clear();
    ninfo.normals.push_back(N1);
    ninfo.normals.push_back(N2);
    ninfo.normals.push_back(N3);

}

} //controller

} //component

}//Sofa

#endif
