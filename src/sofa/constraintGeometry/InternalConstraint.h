#pragma once

#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/type/vector.h>

namespace sofa::constraintGeometry {

class BaseInternalConstraint {
public:

    virtual unsigned size() const = 0;

};

template<class FIRST,class SECOND>
class InternalConstraint : public BaseInternalConstraint, public PairViolationContainer {
public :

    typedef collisionAlgorithm::BaseProximity BaseProximity;
    typedef std::function<core::behavior::ConstraintResolution*(const BaseInternalConstraint *)> ResolutionCreator;

    // create function should be used
    /*!
     * \brief InternalConstraint Constructor
     * \param p1 : 1st proximity
     * \param p2 : second proximity
     * \param normals : ConstraintNormals
     * \param creator : resolutionCreator (factory)
     */
    InternalConstraint(const typename FIRST::SPtr & first, const typename SECOND::SPtr & second, const ConstraintNormal normals, ResolutionCreator creator)
    : m_first(first)
    , m_second(second)
    , m_normals(normals)
    , m_creator(creator)
    , m_cid(0)
    , m_cSetId(0)
    , m_cDirId(0)
    {}

    /*!
     * \brief buildConstraintMatrix
     * Builds proximities p1 & p2's Jacobian Constraints
     * \param cId
     * \param constraintId
     */
    void buildConstraintMatrix(core::MultiMatrixDerivId cId, unsigned int & constraintId) const {
        m_cid = constraintId;

        m_first->buildJacobianConstraint(cId, m_normals.getDirs(),  1.0, constraintId);
        m_second->buildJacobianConstraint(cId, m_normals.getDirs(), -1.0, constraintId);

        constraintId += m_normals.size();
    }

    void getConstraintViolation(linearalgebra::BaseVector *v) const {
        //the ConstraintNormal will compute the all violation (i.e. 1 to 3 depending on the size of the ConstraintNormal)
        m_normals.computeViolations(m_cid, this, v);
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
    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::linearalgebra::BaseVector* lambda) const {
        for (unsigned i=0;i<m_normals.size();i++) {
            m_first->storeLambda(cParams,res,m_cid,i,lambda);
            m_second->storeLambda(cParams,res,m_cid,i,lambda);
        }
    }

    /*!
     * \brief getLambda
     * \param lambda : const BaseVector*
     * \param id : unsigned
     * \return lambda element @ cid+id
     */
    double getLambda(const sofa::linearalgebra::BaseVector* lambda, unsigned id) {
        return lambda->element(m_cid+id);
    }

    void draw(const core::visual::VisualParams* vparams,double scale) const {
        if (m_normals.size()>0) {
            vparams->drawTool()->drawArrow(getFirstProximity()->getPosition(), getFirstProximity()->getPosition() + m_normals.getDirs()[0] * scale, scale * 0.1, type::RGBAColor(1,0,0,1));
//            vparams->drawTool()->drawArrow(m_p2->getPosition(), m_p2->getPosition() - m_normals.m_dirs[0] * scale, scale * 0.1, type::RGBAColor(1,0,0,1));
        }

        if (m_normals.size()>1) {
            vparams->drawTool()->drawArrow(getFirstProximity()->getPosition(), getFirstProximity()->getPosition() + m_normals.getDirs()[1] * scale, scale * 0.1, type::RGBAColor(0,1,0,1));
//            vparams->drawTool()->drawArrow(m_p2->getPosition(), m_p2->getPosition() - m_normals.m_dirs[1] * scale, scale * 0.1, type::RGBAColor(0,1,0,1));
        }

        if (m_normals.size()>2) {
            vparams->drawTool()->drawArrow(getFirstProximity()->getPosition(), getFirstProximity()->getPosition() + m_normals.getDirs()[2] * scale, scale * 0.1, type::RGBAColor(0,0,1,1));
//            vparams->drawTool()->drawArrow(m_p2->getPosition(), m_p2->getPosition() - m_normals.m_dirs[2] * scale, scale * 0.1, type::RGBAColor(0,0,1,1));
        }
    }

    inline typename FIRST::SPtr getFirstProximity() const {
        return m_first;
    }

    inline typename SECOND::SPtr getSecondProximity() const {
        return m_second;
    }

    type::Vector3 getFirstPosition(core::VecCoordId v = core::VecCoordId::position()) const override {
        return m_first->getPosition(v);
    }

    type::Vector3 getSecondPosition(core::VecCoordId v = core::VecCoordId::position()) const override {
        return m_second->getPosition(v);
    }


    const ConstraintNormal getConstraintNormal() const {
        return m_normals;
    }

    unsigned size() const override {
        return m_normals.size();
    }

    unsigned id() const {
        return m_cid;
    }

//    void buildProximityMappingMatrix(core::MultiMatrixDerivId cId, unsigned int & csetId) const {
//        m_cSetId = csetId;

//        sofa::type::vector<type::Vector3> dirs;
//        dirs.clear();
//        dirs.push_back(type::Vector3(1,0,0));
//        dirs.push_back(type::Vector3(0,1,0));
//        dirs.push_back(type::Vector3(0,0,1));

//        m_first->buildJacobianConstraint(cId, dirs,  1.0, csetId*3);
//        m_second->buildJacobianConstraint(cId, dirs, -1.0, csetId*3);

//        csetId ++;
//    }

//    void buildConstraintNormalMatrix(sofa::type::vector<sofa::type::Vector3> & normal, sofa::type::vector<int> & cstInd, unsigned int & cdirId) const {
//        m_cDirId = cdirId;

//        for (unsigned i=0;i<m_normals.size();i++) {
//            normal.push_back(m_normals.m_dirs[i]);
//        }

//        cdirId += m_normals.size();
//        cstInd.push_back(cdirId);
//    }

//    void SetProximityFreePosition(sofa::core::MultiVecDerivId /*pfreeId*/) const {
////        const type::Vector3 & PFree = m_detection.first->getPosition(core::VecCoordId::freePosition());
////        const type::Vector3 & QFree = m_detection.second->getPosition(core::VecCoordId::freePosition());


//    }

    unsigned constraintSetId() const {
        return m_cSetId;
    }

    unsigned constraintDirId() const {
        return m_cDirId;
    }



 protected:
    typename FIRST::SPtr m_first;
    typename SECOND::SPtr m_second;

    ConstraintNormal m_normals;
    ResolutionCreator m_creator;
    mutable unsigned m_cid;

    mutable unsigned m_cSetId;
    mutable unsigned m_cDirId;
};

}
