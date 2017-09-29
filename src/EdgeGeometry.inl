#ifndef SOFA_COMPONENT_EDGEGEOMETRY_INL
#define SOFA_COMPONENT_EDGEGEOMETRY_INL

#include "EdgeGeometry.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include "ConstraintProximity.inl"
#include <sofa/helper/gl/template.h>

namespace sofa {

namespace core {

namespace behavior {

int EdgeGeometry::getNbEdges() const {
    return this->getTopology()->getNbEdges();
}

ConstraintProximityPtr EdgeGeometry::getEdgeProximity(unsigned eid, unsigned p1,double f1,unsigned p2, double f2) const {
    return ConstraintProximityPtr(new EdgeConstraintProximity(this, eid, p1,f1,p2,f2));
}

int EdgeGeometry::getNbElements() const {
    return getNbEdges();
}

ConstraintProximityPtr EdgeGeometry::getElementProximity(unsigned eid) const {
    const sofa::core::topology::BaseMeshTopology::Edge edge = getTopology()->getEdge(eid);
    return getEdgeProximity(eid, edge[0],0.5,edge[1],0.5);
}

void EdgeGeometry::projectPoint(const defaulttype::Vector3 & P, EdgeConstraintProximity *pinfo) const {
    helper::vector<defaulttype::Vector3> pos;
    pinfo->getControlPoints(pos);

    double & fact_u = pinfo->m_fact[0];
    double & fact_v = pinfo->m_fact[1];

    Coord v = pos[1] - pos[0];
    fact_v = dot (P - pos[0],v) / dot (v,v);

    if (fact_v<0.0) fact_v = 0.0;
    else if (fact_v>1.0) fact_v = 1.0;

    fact_u = 1.0-fact_v;
}

void EdgeGeometry::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());

    glDisable(GL_LIGHTING);

    glBegin(GL_LINES);
    glColor4f(d_color.getValue()[0],d_color.getValue()[1],d_color.getValue()[2],d_color.getValue()[3]);
    for(int e=0;e<this->getTopology()->getNbEdges();e++) {
        const sofa::core::topology::BaseMeshTopology::Edge edge= this->getTopology()->getEdge(e);

        //Compute Bezier Positions
        defaulttype::Vector3 p0 = x[edge[0]];
        defaulttype::Vector3 p1 = x[edge[1]];

        helper::gl::glVertexT(p0);
        helper::gl::glVertexT(p1);
    }
    glEnd();
}

} //controller

} //component

}//Sofa

#endif
