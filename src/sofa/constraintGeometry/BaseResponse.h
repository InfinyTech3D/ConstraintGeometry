#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/defaulttype/VecTypes.h>
#include <memory>

namespace sofa {

namespace constraintGeometry {


class ConstraintReponse : public sofa::core::behavior::ConstraintResolution {
public:

    typedef sofa::defaulttype::Vec3dTypes DataTypes;
    typedef DataTypes::VecCoord VecCoord;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;
    typedef DataTypes::VecDeriv VecDeriv;
    typedef DataTypes::MatrixDeriv MatrixDeriv;
    typedef DataTypes::Deriv Deriv1;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef MatrixDeriv::RowIterator MatrixDerivRowIterator;

    ConstraintReponse(const ConstraintNormal & n, collisionAlgorithm::ConstraintProximity::SPtr p1,collisionAlgorithm::ConstraintProximity::SPtr p2)
    : sofa::core::behavior::ConstraintResolution(n.size())
    , m_normals(n)
    , m_p1(p1)
    , m_p2(p2) {}

    inline void buildConstraintMatrix(core::MultiMatrixDerivId cId, unsigned int constraintId) {
        m_p1->buildJacobianConstraint(cId, m_normals, 1.0, constraintId);
        m_p2->buildJacobianConstraint(cId, m_normals, -1.0, constraintId);
    }

    inline void getConstraintViolation(defaulttype::BaseVector *v,unsigned cid) {
        defaulttype::Vector3 PFree = m_p1->getPosition(core::VecCoordId::freePosition());
        defaulttype::Vector3 QFree = m_p2->getPosition(core::VecCoordId::freePosition());
        defaulttype::Vector3 PQFree = PFree - QFree;

        for (unsigned i=0;i<m_normals.size();i++) {
            v->set(cid+i,dot(PQFree,m_normals[i]));
        }
    }

    inline unsigned size() {
        return m_normals.size();
    }

    ConstraintNormal m_normals;
    collisionAlgorithm::ConstraintProximity::SPtr m_p1;
    collisionAlgorithm::ConstraintProximity::SPtr m_p2;
};

}

}
