#ifndef SOFA_COMPONENT_POINTGEOMETRY_INL
#define SOFA_COMPONENT_POINTGEOMETRY_INL

#include "ConstraintGeometry.inl"
#include "PointGeometry.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include "ConstraintProximity.h"

namespace sofa {

namespace core {

namespace behavior {

defaulttype::Vector3 PointGeometry::getNormal(const ConstraintProximity & pinfo) {
    return (pinfo.getPosition() - m_g).normalized();
}

ConstraintProximity PointGeometry::getPointProximity(unsigned eid) {
    ConstraintProximity res(this,eid);
    res.push(eid,1.0);
    return res;
}

double PointGeometry::projectPoint(const defaulttype::Vector3 & T,ConstraintProximity & pinfo) {
    pinfo.push(pinfo.getEid(),1.0);
    return (pinfo.getPosition() - T).norm();
}

int PointGeometry::getNbPoints() {
    return this->getTopology()->getNbPoints();
}

int PointGeometry::size() {
    return getNbPoints();
}

void PointGeometry::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());
    glDisable(GL_LIGHTING);
    glColor4f(d_color.getValue()[0],d_color.getValue()[1],d_color.getValue()[2],d_color.getValue()[3]);
    for (int i=0;i<this->getTopology()->getNbPoints();i++) {
        vparams->drawTool()->drawSphere(x[i],getNorm()*0.01);
//        vparams->drawTool()->drawArrow(x[i],x[i]+getNormal(getPointProximity(i))*norm*0.5,norm*0.01,defaulttype::Vec<4,float>(1.0f,0.0f,0.0f,1.0f));
    }
}

} //controller

} //component

}//Sofa

#endif
