#pragma once

#include <sofa/constraintGeometry/normals/ConstraintNormal.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/helper/vector.h>

namespace sofa {

namespace constraintGeometry {

class InternalConstraint {
public :

    typedef std::function<core::behavior::ConstraintResolution*(const InternalConstraint *)> ResolutionCreator;

    // create function should be used
    /*!
     * \brief InternalConstraint Constructor
     * \param p1 : 1st proximity
     * \param p2 : second proximity
     * \param normals : ConstraintNormals
     * \param creator : resolutionCreator (factory)
     */
    InternalConstraint(collisionAlgorithm::BaseProximity::SPtr p1,collisionAlgorithm::BaseProximity::SPtr p2,const ConstraintNormal normals, ResolutionCreator creator)
    : m_p1(p1)
    , m_p2(p2)
    , m_normals(normals)
    , m_creator(creator)
    , m_cid(0) {}

    /*!
     * \brief buildConstraintMatrix
     * Builds proximities p1 & p2's Jacobian Constraints
     * \param cId
     * \param constraintId
     */
    void buildConstraintMatrix(core::MultiMatrixDerivId cId, unsigned int & constraintId) const {
        m_cid = constraintId;

        m_p1->buildJacobianConstraint(cId, m_normals.m_dirs,  1.0, constraintId);
        m_p2->buildJacobianConstraint(cId, m_normals.m_dirs, -1.0, constraintId);

        constraintId += m_normals.size();
    }

    void getConstraintViolation(defaulttype::BaseVector *v) const {
        defaulttype::Vector3 PFree = m_p1->getPosition(core::VecCoordId::freePosition());
        defaulttype::Vector3 QFree = m_p2->getPosition(core::VecCoordId::freePosition());
        defaulttype::Vector3 PQFree = PFree - QFree;

        for (unsigned i=0;i<m_normals.size();i++) {
            v->set(m_cid+i,dot(PQFree,m_normals.m_dirs[i]));
        }
    }

    /*!
     * \brief createConstraintResolution, creates ConstraintResolution using creator's factory
     * \return ConstraintResolution*
     */
    core::behavior::ConstraintResolution* createConstraintResolution() const {
        return m_creator(this);
    }

    /*!
     * \brief storeLambda, calls storeLambda on p1 and p2 proximities' storeLambda
     * \param cParams
     * \param res
     * \param lambda
     */
    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::defaulttype::BaseVector* lambda) const {
        for (unsigned i=0;i<m_normals.size();i++) {
            m_p1->storeLambda(cParams,res,m_cid+i,lambda);
            m_p2->storeLambda(cParams,res,m_cid+i,lambda);
        }
    }

    /*!
     * \brief getLambda
     * \param lambda : const BaseVector*
     * \param id : unsigned
     * \return lambda element @ cid+id
     */
    double getLambda(const sofa::defaulttype::BaseVector* lambda, unsigned id) {
        return lambda->element(m_cid+id);
    }

    void draw(const core::visual::VisualParams* vparams,double scale) const {
        if (m_normals.size()>0) {
            vparams->drawTool()->drawArrow(m_p1->getPosition(), m_p1->getPosition() + m_normals.m_dirs[0] * scale, scale * 0.1, defaulttype::Vector4(1,0,0,1));
//            vparams->drawTool()->drawArrow(m_p2->getPosition(), m_p2->getPosition() - m_normals.m_dirs[0] * scale, scale * 0.1, defaulttype::Vector4(1,0,0,1));
        }

        if (m_normals.size()>1) {
            vparams->drawTool()->drawArrow(m_p1->getPosition(), m_p1->getPosition() + m_normals.m_dirs[1] * scale, scale * 0.1, defaulttype::Vector4(0,1,0,1));
//            vparams->drawTool()->drawArrow(m_p2->getPosition(), m_p2->getPosition() - m_normals.m_dirs[1] * scale, scale * 0.1, defaulttype::Vector4(0,1,0,1));
        }

        if (m_normals.size()>2) {
            vparams->drawTool()->drawArrow(m_p1->getPosition(), m_p1->getPosition() + m_normals.m_dirs[2] * scale, scale * 0.1, defaulttype::Vector4(0,0,1,1));
//            vparams->drawTool()->drawArrow(m_p2->getPosition(), m_p2->getPosition() - m_normals.m_dirs[2] * scale, scale * 0.1, defaulttype::Vector4(0,0,1,1));
        }
    }

    collisionAlgorithm::BaseProximity::SPtr getFirstProximity() const {
        return m_p1;
    }

    collisionAlgorithm::BaseProximity::SPtr getSecondProximity() const {
        return m_p2;
    }

    unsigned size() const {
        return m_normals.size();
    }

    unsigned id() const {
        return m_cid;
    }

 protected:
    collisionAlgorithm::BaseProximity::SPtr m_p1;
    collisionAlgorithm::BaseProximity::SPtr m_p2;
    ConstraintNormal m_normals;
    ResolutionCreator m_creator;
    mutable unsigned m_cid;
};

}

}
