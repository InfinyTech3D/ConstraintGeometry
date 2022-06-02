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
#include <sofa/constraintGeometry/normalHandler/VectorPointNormalHandler.h>

namespace sofa::constraintGeometry {

template<class DataTypes>
class VectorPointConstraintProximity : public ConstraintProximity {
public:

    typedef std::shared_ptr<VectorPointConstraintProximity> SPtr;

    VectorPointConstraintProximity(typename collisionAlgorithm::MechanicalProximity<DataTypes>::SPtr prox, const type::vector<type::Vector3> & normals)
    : m_prox(prox), m_normals(normals) {}


    type::Vector3 getNormal() override {
        type::Vector3 N;

        if (m_prox->getPId()>=m_normals.size()) {
            std::cerr << "Wrong id in VectorPointNormalHandler" << std::endl;
            return N;
        }

        N = m_normals[m_prox->getPId()];
        return N;

    }

    collisionAlgorithm::BaseProximity::SPtr getProximity() const override { return m_prox; }


private:
    typename collisionAlgorithm::MechanicalProximity<DataTypes>::SPtr m_prox;
    const type::vector<type::Vector3> & m_normals;
};

}
