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

    ConstraintProximity::SPtr buildConstraintProximity(collisionAlgorithm::BaseProximity::SPtr prox) override {
        if (collisionAlgorithm::EdgeProximity::SPtr eprox = std::dynamic_pointer_cast<collisionAlgorithm::EdgeProximity>(prox)) {
            //TODO : return NEW EDGE CSTPROX
            return ConstraintProximity::SPtr(new EdgeConstraintProximity(eprox));
        }

        return NULL;
    }

    const std::type_info & getTypeInfo() override { return typeid(EdgeNormalHandler); }

};

}
