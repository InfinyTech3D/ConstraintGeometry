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

ConstraintProximityPtr PointGeometry::getPointProximity(unsigned eid) const {
    return ConstraintProximityPtr(new PointConstraintProximity(this,eid));
}

ConstraintProximityPtr PointGeometry::projectPoint(const defaulttype::Vector3 & /*T*/,unsigned eid) const {
    return getPointProximity(eid);
}

int PointGeometry::getNbElements() const {
    return this->getTopology()->getNbPoints();
}

void PointGeometry::draw(const core::visual::VisualParams * vparams) {
    if (!vparams->displayFlags().getShowCollisionModels()) return;

    double norm = (this->f_bbox.getValue().maxBBox() - this->f_bbox.getValue().minBBox()).norm();

    glDisable(GL_LIGHTING);
    glColor4f(d_color.getValue()[0],d_color.getValue()[1],d_color.getValue()[2],d_color.getValue()[3]);
    for (int i=0;i<this->getNbElements();i++) {
        vparams->drawTool()->drawSphere(this->getPointProximity(i)->getPosition(),norm*0.01);
    }
}

} //controller

} //component

}//Sofa

#endif
