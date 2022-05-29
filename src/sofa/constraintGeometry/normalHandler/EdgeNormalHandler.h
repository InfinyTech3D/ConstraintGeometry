#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>

namespace sofa::constraintGeometry {

class EdgeNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(EdgeNormalHandler, BaseNormalHandler);

    bool getNormal(collisionAlgorithm::BaseProximity::SPtr prox,type::Vector3 & N) override {
        if (collisionAlgorithm::EdgeProximity::SPtr tprox = std::dynamic_pointer_cast<collisionAlgorithm::EdgeProximity>(prox)) {
            N = (tprox->element()->getP1()->getPosition() - tprox->element()->getP0()->getPosition()).normalized();
            return true;
        }

        std::cerr << "Error the proximity is no a TriangleProximity in GouraudTriangleNormalHandler " << std::endl;
        return false;
    }

    void prepareDetection() override {}

};

}
