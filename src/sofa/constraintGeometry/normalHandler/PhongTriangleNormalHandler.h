#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/constraintGeometry/constraintProximities/PhongConstraintProximity.h>

namespace sofa::constraintGeometry {

class PhongTriangleNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(PhongTriangleNormalHandler, BaseNormalHandler);


    //THIS FUNCTION SHOULD BE MOVE IN THE CST PHONG PROXIMIT
    //IT SHOULD ALSO BE REMOVE FROM BASENORMALHANDLER (virtual pure)


    //THIS FUNCTION SHOULD BE REMOVE FROM BASENORMALHANDLER (virtual pure)
    void prepareDetection() override {}

    const std::type_info & getTypeInfo() override { return typeid(PhongTriangleNormalHandler); }

    ConstraintProximity::SPtr buildConstraintProximity(collisionAlgorithm::BaseProximity::SPtr prox) {
        if (collisionAlgorithm::TriangleProximity::SPtr tprox = std::dynamic_pointer_cast<collisionAlgorithm::TriangleProximity>(prox)) {
            //TODO : return NEW PHONG CSTPROX
            return ConstraintProximity::SPtr(new PhongConstraintProximity(tprox));
        }

        return NULL;
    }
};



}
