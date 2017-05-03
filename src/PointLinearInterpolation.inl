#ifndef SOFA_COMPONENT_POINTLINEARINTERPOLATION_INL
#define SOFA_COMPONENT_POINTLINEARINTERPOLATION_INL

#include "PointLinearInterpolation.h"
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
ConstraintProximity PointLinearInterpolation<DataTypes>::findClosestProximity(const defaulttype::Vector3 & P) {
    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());

    unsigned min_eid = 0;
    double minDist = 0;

    for(unsigned e=0;e<x.size();e++) {
        double dist = (x[e]-P).norm();

        if ((e==0) || (dist < minDist)) {
            min_eid = e;

            minDist = dist;
        }
    }

    return this->getPointProximity(min_eid);
}

} //controller

} //component

}//Sofa

#endif
