#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa::constraintGeometry {

class GouraudTriangleNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(GouraudTriangleNormalHandler, BaseNormalHandler);

    bool getNormal(collisionAlgorithm::BaseProximity::SPtr prox,type::Vector3 & N) override {
        if (collisionAlgorithm::TriangleProximity::SPtr tprox = std::dynamic_pointer_cast<collisionAlgorithm::TriangleProximity>(prox)) {
            N = tprox->element()->getTriangleInfo().N;
            return true;
        }

        std::cerr << "Error the proximity is no a TriangleProximity in GouraudTriangleNormalHandler " << std::endl;
        return false;
    }

    void prepareDetection() override {}

    const std::type_info & getTypeInfo() override { return typeid(GouraudTriangleNormalHandler); }
};

}
