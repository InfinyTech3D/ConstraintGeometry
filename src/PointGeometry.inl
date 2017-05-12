#ifndef SOFA_COMPONENT_POINTGEOMETRY_INL
#define SOFA_COMPONENT_POINTGEOMETRY_INL

#include "ConstraintGeometry.inl"
#include "PointGeometry.h"
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

defaulttype::Vector3 PointGeometry::getNormal(const ConstraintProximity & /*pinfo*/) {
    return defaulttype::Vector3();
}

ConstraintProximity PointGeometry::getPointProximity(unsigned eid) {
    ConstraintProximity res(this,eid);
    res.push(eid,1.0);
    return res;
}

ConstraintProximity PointGeometry::projectPoint(unsigned eid,const defaulttype::Vector3 & /*T*/) {
    return getPointProximity(eid);
}

unsigned PointGeometry::getNbElements() {
    return this->getTopology()->getNbPoints();
}

} //controller

} //component

}//Sofa

#endif
