#ifndef SOFA_COMPONENT_EDGEGEOMETRY_INL
#define SOFA_COMPONENT_EDGEGEOMETRY_INL

#include "EdgeGeometry.h"
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


unsigned EdgeGeometry::getNbElements() {
    return this->getTopology()->getNbEdges();
}

ConstraintProximity EdgeGeometry::projectPoint(unsigned eid,const defaulttype::Vector3 & P) {
    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());

    double fact_u;
    double fact_v;

    topology::Edge edge = this->getTopology()->getEdge(eid);

    Coord v = x[edge[1]] - x[edge[0]];
    fact_v = dot (P - x[edge[0]],v) / dot (v,v);

    if (fact_v<0.0) fact_v = 0.0;
    else if (fact_v>1.0) fact_v = 1.0;

    fact_u = 1.0-fact_v;

    return this->getEdgeProximity(eid,fact_u,fact_v);
}

ConstraintProximity EdgeGeometry::getEdgeProximity(unsigned eid, double fact_u,double fact_v) {
    ConstraintProximity res(this,eid);
    topology::Edge edge = this->getTopology()->getEdge(eid);

    res.push(edge[0],fact_u);
    res.push(edge[1],fact_v);

    return res;
}

defaulttype::Vector3 EdgeGeometry::getNormal(const ConstraintProximity & pinfo) {
    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());

    topology::Edge edge = this->getTopology()->getEdge(pinfo.getEid());

    Vector3 v = x[edge[1]] - x[edge[0]];

    return v.normalized();
}


void EdgeGeometry::draw(const core::visual::VisualParams * /*vparams*/) {

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
