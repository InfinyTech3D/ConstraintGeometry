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

void PointGeometry::prepareDetection() {
    m_g = Vector3(0,0,0);

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());
    for (unsigned i=0;i<x.size();i++) {
        m_g += x[i];
    }

    m_g *= 1.0/x.size();
}


double PointGeometry::projectPoint(const defaulttype::Vector3 & T,ConstraintProximity & pinfo) {
    pinfo.push(pinfo.getEid(),1.0);
    return (pinfo.getPosition() - T).norm();
}

int PointGeometry::getNbElements() {
    return this->getTopology()->getNbPoints();
}

void PointGeometry::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());

    double norm = 0.0;
    for (unsigned i=0;i<x.size();i++) {
        norm += (m_g - x[i]).norm();
    }
    norm*=1.0/x.size();

    glColor3f(0.9,0.46,0);
    for (int i=0;i<this->getTopology()->getNbPoints();i++) {
        vparams->drawTool()->drawSphere(x[i],norm*0.05);
        vparams->drawTool()->drawArrow(x[i],x[i]+getNormal(getPointProximity(i))*norm*0.5,norm*0.01,defaulttype::Vec<4,float>(1.0f,0.0f,0.0f,1.0f));
    }
}

} //controller

} //component

}//Sofa

#endif
