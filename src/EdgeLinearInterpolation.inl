#ifndef SOFA_COMPONENT_EDGELINEARINTERPOLATION_INL
#define SOFA_COMPONENT_EDGELINEARINTERPOLATION_INL

#include "EdgeLinearInterpolation.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaOpenglVisual/OglModel.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaConstraint/BilateralInteractionConstraint.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include "ConstraintProximity.h"

namespace sofa {

namespace core {

namespace behavior {


template<class DataTypes>
ConstraintProximity EdgeLinearInterpolation<DataTypes>::findClosestProximity(const defaulttype::Vector3 & P) {
    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());

    unsigned min_eid = 0;
    double min_fact[2] = {0,0};
    double minDist = 0;

    for(int e=0;e<this->getTopology()->getNbEdges();e++) {
        double fact_u;
        double fact_v;

        topology::Edge edge = this->getTopology()->getEdge(e);

        Coord v = x[edge[1]] - x[edge[0]];
        fact_v = dot (P - x[edge[0]],v) / dot (v,v);

        if (fact_v<0.0) fact_v = 0.0;
        else if (fact_v>1.0) fact_v = 1.0;

        fact_u = 1.0-fact_v;

        Coord Q = x[edge[0]] * fact_u + x[edge[1]] * fact_v;

        double dist = (Q-P).norm();

        if ((e==0) || (dist < minDist)) {
            min_fact[0] = fact_u;
            min_fact[1] = fact_v;

            min_eid = e;

            minDist = dist;
        }
    }

    return this->getEdgeProximity(min_eid,min_fact[0],min_fact[1]);
}


template<class DataTypes>
void EdgeLinearInterpolation<DataTypes>::draw(const core::visual::VisualParams * /*vparams*/) {

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());

    glDisable(GL_LIGHTING);

    glBegin(GL_LINES);
    glColor3f(1,0.5,0);
    for(int e=0;e<this->getTopology()->getNbEdges();e++) {
        const topology::Edge edge= this->getTopology()->getEdge(e);

        //Compute Bezier Positions
        Vector3 p0 = x[edge[0]];
        Vector3 p1 = x[edge[1]];

        helper::gl::glVertexT(p0);
        helper::gl::glVertexT(p1);
    }
    glEnd();
}

} //controller

} //component

}//Sofa

#endif
