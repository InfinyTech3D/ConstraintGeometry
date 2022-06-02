#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/constraintGeometry/constraintProximities/GouraudConstraintProximity.h>

namespace sofa::constraintGeometry {

class GouraudTriangleNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(GouraudTriangleNormalHandler, BaseNormalHandler);


    void prepareDetection() override {}

    static ConstraintProximity::SPtr buildConstraintProximity(collisionAlgorithm::TriangleProximity::SPtr tprox) {
        return ConstraintProximity::SPtr(new GouraudConstraintProximity(tprox));
    }

    const std::type_info & getTypeInfo() override { return typeid(GouraudTriangleNormalHandler); }
};

}
