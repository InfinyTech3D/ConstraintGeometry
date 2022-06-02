#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/constraintGeometry/constraintProximities/EdgeConstraintProximity.h>

namespace sofa::constraintGeometry {

class EdgeNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(EdgeNormalHandler, BaseNormalHandler);


    void prepareDetection() override {}

    static ConstraintProximity::SPtr buildConstraintProximity(EdgeNormalHandler *, collisionAlgorithm::EdgeProximity::SPtr prox) {
        return ConstraintProximity::SPtr(new EdgeConstraintProximity(prox));
    }

    const std::type_info & getTypeInfo() override { return typeid(EdgeNormalHandler); }

};

}
