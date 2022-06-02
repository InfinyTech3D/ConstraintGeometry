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
#include <sofa/constraintGeometry/normalHandler/EdgeNormalHandler.h>

namespace sofa::constraintGeometry {

class EdgeConstraintProximity : public ConstraintProximity {
public:

    typedef std::shared_ptr<EdgeConstraintProximity> SPtr;

    EdgeConstraintProximity(collisionAlgorithm::EdgeProximity::SPtr prox)
    : m_prox(prox) {}


    type::Vector3 getNormal() override {
        type::Vector3 N = (m_prox->element()->getP1()->getPosition() - m_prox->element()->getP0()->getPosition()).normalized();
        return N;
    }

    collisionAlgorithm::BaseProximity::SPtr getProximity() const override { return m_prox; }


private:
    collisionAlgorithm::EdgeProximity::SPtr m_prox;
};

}
