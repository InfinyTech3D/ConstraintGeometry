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

defaulttype::Vector3 PointGeometry::getNormal(const ConstraintProximity & /*pinfo*/) {
    return defaulttype::Vector3();
}

ConstraintProximity PointGeometry::getPointProximity(unsigned eid) {
    ConstraintProximity res(this,eid);
    res.push(eid,1.0);
    return res;
}

double PointGeometry::projectPoint(unsigned eid,const defaulttype::Vector3 & T,ConstraintProximity & pinfo) {
    pinfo = getPointProximity(eid);
    return (pinfo.getPosition() - T).norm();
}

int PointGeometry::getNbElements() {
    return this->getTopology()->getNbPoints();
}

void PointGeometry::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());

    glColor3f(0.9,0.46,0);
    for (int i=0;i<this->getTopology()->getNbPoints();i++) {
        vparams->drawTool()->drawSphere(x[i],0.001);
    }
}

} //controller

} //component

}//Sofa

#endif
