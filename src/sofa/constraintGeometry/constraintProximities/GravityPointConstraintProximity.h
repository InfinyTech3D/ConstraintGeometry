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
#include <sofa/constraintGeometry/normalHandler/GravityPointNormalHandler.h>

namespace sofa::constraintGeometry {

class GravityPointConstraintProximity : public ConstraintProximity {
public:

    typedef std::shared_ptr<GravityPointConstraintProximity> SPtr;

    GravityPointConstraintProximity(collisionAlgorithm::PointProximity::SPtr prox, type::Vector3 gcenter)
    : m_prox(prox), m_gcenter(gcenter) {}


    type::Vector3 getNormal() override {
        type::Vector3 N = (m_prox->getPosition() - m_gcenter).normalized();
        return N;
    }

    collisionAlgorithm::BaseProximity::SPtr getProximity() const override { return m_prox; }


private:
    collisionAlgorithm::PointProximity::SPtr m_prox;
    type::Vector3 m_gcenter;
};

}
