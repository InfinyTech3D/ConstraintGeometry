#pragma once

#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/helper/vector.h>

namespace sofa {

namespace constraintGeometry {

class InternalConstraint {
public:
    InternalConstraint(collisionAlgorithm::BaseProximity::SPtr p1,collisionAlgorithm::BaseProximity::SPtr p2,ConstraintNormal normals,sofa::core::behavior::ConstraintResolution * resolution)
    : m_p1(p1)
    , m_p2(p2)
    , m_resolution(resolution)
    , m_normals(normals) {
        if (m_normals.size() != m_resolution->getNbLines()) {
            std::cerr << "ERROR you provided a ConstraintResolution and a Constraint normals with different size" << std::endl;

//                m_normals = normals.resize()
        }
    }

    void buildConstraintMatrix(core::MultiMatrixDerivId cId, unsigned int & constraintId) {
        m_p1->buildJacobianConstraint(cId, m_normals.m_dirs,  1.0, constraintId);
        m_p2->buildJacobianConstraint(cId, m_normals.m_dirs, -1.0, constraintId);

        constraintId += m_normals.size();
    }

    void getConstraintViolation(defaulttype::BaseVector *v,unsigned & cid) {
        defaulttype::Vector3 PFree = m_p1->getPosition(core::VecCoordId::freePosition());
        defaulttype::Vector3 QFree = m_p2->getPosition(core::VecCoordId::freePosition());
        defaulttype::Vector3 PQFree = PFree - QFree;

        for (unsigned i=0;i<m_normals.size();i++) {
            v->set(cid+i,dot(PQFree,m_normals.m_dirs[i]));
        }

        cid += m_normals.size();
    }

    void getConstraintResolution(std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned & offset) {
        resTab[offset] = m_resolution;
        offset += m_normals.size();
    }

    void draw(const core::visual::VisualParams* vparams,double scale) {
        if (m_normals.size()>0)
            vparams->drawTool()->drawArrow(m_p1->getPosition(), m_p1->getPosition() + m_normals.m_dirs[0] * scale, scale * 0.1, defaulttype::Vector4(1,0,0,1));

        if (m_normals.size()>1)
            vparams->drawTool()->drawArrow(m_p1->getPosition(), m_p1->getPosition() + m_normals.m_dirs[1] * scale, scale * 0.1, defaulttype::Vector4(0,1,0,1));

        if (m_normals.size()>2)
            vparams->drawTool()->drawArrow(m_p1->getPosition(), m_p1->getPosition() + m_normals.m_dirs[2] * scale, scale * 0.1, defaulttype::Vector4(0,0,1,1));
    }

    collisionAlgorithm::BaseProximity::SPtr getFirstProximity() const {
        return m_p1;
    }

    collisionAlgorithm::BaseProximity::SPtr getSecondProximity() const {
        return m_p2;
    }

 private:
    collisionAlgorithm::BaseProximity::SPtr m_p1;
    collisionAlgorithm::BaseProximity::SPtr m_p2;
    sofa::core::behavior::ConstraintResolution * m_resolution;
    ConstraintNormal m_normals;
};

}

}
