#ifndef SOFA_COMPONENT_POINTGEOMETRY_INL
#define SOFA_COMPONENT_POINTGEOMETRY_INL

#include "PointGeometry.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaBaseMechanics/MechanicalObject.h>

namespace sofa {

namespace core {

namespace behavior {

ConstraintProximityPtr PointGeometry::getPointProximity(unsigned eid) const {
    return ConstraintProximityPtr(new PointConstraintProximity(this,eid));
}

int PointGeometry::getNbPoints() const {
    return this->getTopology()->getNbPoints();
}

int PointGeometry::getNbElements() const {
    return getNbPoints();
}

ConstraintProximityPtr PointGeometry::getElementProximity(unsigned eid) const {
    return getPointProximity(eid);
}


void PointGeometry::draw(const core::visual::VisualParams * vparams) {
    if (!vparams->displayFlags().getShowCollisionModels()) return;

    double norm = (this->f_bbox.getValue().maxBBox() - this->f_bbox.getValue().minBBox()).norm();

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());

    glDisable(GL_LIGHTING);
    glColor4f(d_color.getValue()[0],d_color.getValue()[1],d_color.getValue()[2],d_color.getValue()[3]);
    for (int i=0;i<this->getNbPoints();i++) {
        vparams->drawTool()->drawSphere(x[i],norm*0.01);
    }
}

} //controller

} //component

}//Sofa

#endif
