#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>

namespace sofa::constraintGeometry {

class PhongTriangleNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(PhongTriangleNormalHandler, BaseNormalHandler);

    bool getNormal(collisionAlgorithm::BaseProximity::SPtr prox,type::Vector3 & N) override {
        collisionAlgorithm::TriangleProximity::SPtr tprox = std::dynamic_pointer_cast<collisionAlgorithm::TriangleProximity>(prox);

        if (tprox==NULL) {
            std::cerr << "Error the proximity is no a TriangleProximity in GouraudTriangleNormalHandler " << std::endl;
            return false;
        }

        auto element = tprox->element();

        collisionAlgorithm::PointElement::SPtr p0 = element->pointElements()[0];
        collisionAlgorithm::PointElement::SPtr p1 = element->pointElements()[1];
        collisionAlgorithm::PointElement::SPtr p2 = element->pointElements()[2];

        type::Vector3 N0_point;
        for (auto it = p0->triangleAround().cbegin();it!=p0->triangleAround().cend();it++) {
            N0_point+=(*it)->getTriangleInfo().N;
        }

        type::Vector3 N1_point;
        for (auto it = p1->triangleAround().cbegin();it!=p1->triangleAround().cend();it++) {
            N1_point+=(*it)->getTriangleInfo().N;
        }

        type::Vector3 N2_point;
        for (auto it = p2->triangleAround().cbegin();it!=p2->triangleAround().cend();it++) {
            N2_point+=(*it)->getTriangleInfo().N;
        }

        N = N0_point.normalized() * tprox->f0() +
            N1_point.normalized() * tprox->f1() +
            N2_point.normalized() * tprox->f2();

        return true;
    }

    void prepareDetection() override {}

};



}
