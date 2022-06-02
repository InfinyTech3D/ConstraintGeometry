#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/constraintGeometry/constraintProximities/PhongConstraintProximity.h>

namespace sofa::constraintGeometry {

class PhongTriangleNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(PhongTriangleNormalHandler, BaseNormalHandler);

    void prepareDetection() override {}

    const std::type_info & getTypeInfo() override { return typeid(PhongTriangleNormalHandler); }

    static ConstraintProximity::SPtr buildConstraintProximity(PhongTriangleNormalHandler *, collisionAlgorithm::TriangleProximity::SPtr tprox) {
        return ConstraintProximity::SPtr(new PhongConstraintProximity(tprox));
    }
};



}
