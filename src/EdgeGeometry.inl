#ifndef SOFA_COMPONENT_EDGEGEOMETRY_INL
#define SOFA_COMPONENT_EDGEGEOMETRY_INL

#include "EdgeGeometry.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include "ConstraintProximity.h"
#include <sofa/helper/gl/template.h>

namespace sofa {

namespace core {

namespace behavior {


int EdgeGeometry::getNbEdges() const {
    return this->getTopology()->getNbEdges();
}

int EdgeGeometry::getNbElements() const{
    return getNbEdges();
}

ConstraintProximityPtr EdgeGeometry::projectPoint(const defaulttype::Vector3 & P,unsigned eid) const {
    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());

    double fact_u;
    double fact_v;

    sofa::core::topology::BaseMeshTopology::Edge edge = this->getTopology()->getEdge(eid);

    Coord v = x[edge[1]] - x[edge[0]];
    fact_v = dot (P - x[edge[0]],v) / dot (v,v);

    if (fact_v<0.0) fact_v = 0.0;
    else if (fact_v>1.0) fact_v = 1.0;

    fact_u = 1.0-fact_v;

    return ConstraintProximityPtr(new EdgeConstraintProximity(this, edge[0],fact_u,edge[1],fact_v));
}

void EdgeGeometry::draw(const core::visual::VisualParams * /*vparams*/) {

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
