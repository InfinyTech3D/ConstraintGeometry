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

class InternalConstraint : public BaseInternalConstraint {
public :

    typedef collisionAlgorithm::BaseBaseProximity FIRST;
    typedef collisionAlgorithm::BaseBaseProximity SECOND;

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
    : m_creator(creator)
    , m_cid(0)
    , m_cSetId(0)
    , m_cDirId(0)
    {
        m_pairs.push_back(std::pair<const typename FIRST::SPtr, const typename SECOND::SPtr>(first, second));
        m_vecNormals.push_back(normals);
    }

    InternalConstraint(std::vector<std::pair<const typename FIRST::SPtr, const typename SECOND::SPtr>> & pairs, std::vector<ConstraintNormal> & vecNormals, ResolutionCreator creator)
    : m_pairs(pairs)
    , m_vecNormals(vecNormals)
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

        for (unsigned i=0; i<m_vecNormals.size(); i++) {
            m_pairs[i].first->buildJacobianConstraint(cId, m_vecNormals[i].getDirs(),  1.0, constraintId);
            m_pairs[i].second->buildJacobianConstraint(cId, m_vecNormals[i].getDirs(), -1.0, constraintId);

            constraintId += m_vecNormals[i].size();
        }
    }

    void getConstraintViolation(linearalgebra::BaseVector *v) const {
        //the ConstraintNormal will compute the all violation (i.e. 1 to 3 depending on the size of the ConstraintNormal)
        unsigned int lcid = m_cid;
        for (unsigned i=0; i<m_vecNormals.size(); i++) {
            m_vecNormals[i].computeViolations(lcid, m_pairs[i].first, m_pairs[i].second, v);
            /*++ lcid;*/  lcid += m_vecNormals[i].size();
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
    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::linearalgebra::BaseVector* lambda) const {
        unsigned int lcid=m_cid;
        for (unsigned k=0; k<m_vecNormals.size(); k++) {
            for (unsigned i=0;i<m_vecNormals[k].size();i++) {
                m_pairs[k].first->storeLambda(cParams,res,/*m_cid*/lcid,i,lambda);
                m_pairs[k].second->storeLambda(cParams,res,/*m_cid*/lcid,i,lambda);
            }
            lcid += m_vecNormals[k].size();
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
        for (unsigned i=0; i<m_vecNormals.size(); i++) {
            draw(vparams,scale,i);
        }
    }

    void draw(const core::visual::VisualParams* vparams,double scale, unsigned i) const {
        if (m_vecNormals[i].size()>0) {
            vparams->drawTool()->drawArrow(m_pairs[i].first->getPosition(), m_pairs[i].first->getPosition() + m_vecNormals[i].getDirs()[0] * scale, scale * 0.1, type::RGBAColor(1,0,0,1));
            vparams->drawTool()->drawArrow(m_pairs[i].second->getPosition(), m_pairs[i].second->getPosition() - m_vecNormals[i].getDirs()[0] * scale, scale * 0.1, type::RGBAColor(1,0,0,1));
        }

        if (m_vecNormals[i].size()>1) {
            vparams->drawTool()->drawArrow(m_pairs[i].first->getPosition(), m_pairs[i].first->getPosition() + m_vecNormals[i].getDirs()[1] * scale, scale * 0.1, type::RGBAColor(0,1,0,1));
            vparams->drawTool()->drawArrow(m_pairs[i].second->getPosition(), m_pairs[i].second->getPosition() - m_vecNormals[i].getDirs()[1] * scale, scale * 0.1, type::RGBAColor(0,1,0,1));
        }

        if (m_vecNormals[i].size()>2) {
            vparams->drawTool()->drawArrow(m_pairs[i].first->getPosition(), m_pairs[i].first->getPosition() + m_vecNormals[i].getDirs()[2] * scale, scale * 0.1, type::RGBAColor(0,0,1,1));
            vparams->drawTool()->drawArrow(m_pairs[i].second->getPosition(), m_pairs[i].second->getPosition() - m_vecNormals[i].getDirs()[2] * scale, scale * 0.1, type::RGBAColor(0,0,1,1));
        }
    }


    const std::vector<ConstraintNormal> & getConstraintNormal() const {
        return m_vecNormals;
    }

    const std::vector<std::pair<const typename FIRST::SPtr, const typename SECOND::SPtr>> & getPairs() const {
        return m_pairs;
    }

    unsigned size() const override {
        unsigned lcid=0;
        for(unsigned i=0; i<m_vecNormals.size(); i++) {
            lcid += m_vecNormals[i].size();
        }
        return lcid;
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
//    typename FIRST::SPtr m_first;
//    typename SECOND::SPtr m_second;

    std::vector<std::pair<const typename FIRST::SPtr, const typename SECOND::SPtr>> m_pairs;

    std::vector<ConstraintNormal> m_vecNormals;
    ResolutionCreator m_creator;
    mutable unsigned m_cid;

    mutable unsigned m_cSetId;
    mutable unsigned m_cDirId;
};

}
