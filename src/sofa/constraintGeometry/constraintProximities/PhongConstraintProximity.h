#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <sofa/constraintGeometry/InternalConstraint.h>
#include <sofa/constraintGeometry/ConstraintResponse.h>
#include <sofa/constraintGeometry/ConstraintDirection.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/constraintGeometry/normalHandler/PhongTriangleNormalHandler.h>

namespace sofa::constraintGeometry {

class PhongConstraintProximity : public ConstraintProximity {
public:

    typedef std::shared_ptr<PhongConstraintProximity> SPtr;

    PhongConstraintProximity(collisionAlgorithm::TriangleProximity::SPtr prox)
    : m_prox(prox) {}



    type::Vector3 getNormal() override {
        auto element = m_prox->element();

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

        type::Vector3 N = N0_point.normalized() * m_prox->f0() +
            N1_point.normalized() * m_prox->f1() +
            N2_point.normalized() * m_prox->f2();

        return N;
    }

    collisionAlgorithm::BaseProximity::SPtr getProximity() const override { return m_prox; }


private:
    collisionAlgorithm::TriangleProximity::SPtr m_prox;
};

}
