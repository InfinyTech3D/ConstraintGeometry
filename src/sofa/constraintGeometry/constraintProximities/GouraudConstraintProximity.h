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
#include <sofa/constraintGeometry/normalHandler/GouraudTriangleNormalHandler.h>

namespace sofa::constraintGeometry {

class GouraudConstraintProximity : public ConstraintProximity {
public:

    typedef std::shared_ptr<GouraudConstraintProximity> SPtr;

    GouraudConstraintProximity(collisionAlgorithm::TriangleProximity::SPtr prox)
    : m_prox(prox) {}


    type::Vector3 getNormal() override {
        return m_prox->element()->getTriangleInfo().N;
    }

    collisionAlgorithm::BaseProximity::SPtr getProximity() const override { return m_prox; }


private:
    collisionAlgorithm::TriangleProximity::SPtr m_prox;
};

}
