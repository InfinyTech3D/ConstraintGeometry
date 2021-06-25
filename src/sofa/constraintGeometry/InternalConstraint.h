#pragma once

#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/helper/vector.h>

namespace sofa {

namespace constraintGeometry {

class InternalConstraint {
public :

    typedef std::function<core::behavior::ConstraintResolution*(const InternalConstraint &)> ResolutionCreator;

    // create function should be used
    /*!
     * \brief InternalConstraint Constructor
     * \param p1 : 1st proximity
     * \param p2 : second proximity
     * \param normals : ConstraintNormals
     * \param creator : resolutionCreator (factory)
     */
    InternalConstraint(collisionAlgorithm::PairDetection detection,const ConstraintNormal normals, ResolutionCreator creator)
    : m_detection(detection)
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

        m_detection.first->buildJacobianConstraint(cId, m_normals.m_dirs,  1.0, constraintId);
        m_detection.second->buildJacobianConstraint(cId, m_normals.m_dirs, -1.0, constraintId);

        constraintId += m_normals.size();
    }

    void getConstraintViolation(defaulttype::BaseVector *v) const {
        //for (unsigned i=0;i<m_normals.size();i++) {
            m_normals.computeViolations(m_cid, m_detection, v);
        //}
    }

    /*!
     * \brief createConstraintResolution, creates ConstraintResolution using creator's factory
     * \return ConstraintResolution*
     */
    core::behavior::ConstraintResolution* createConstraintResolution() const {
        return m_creator(*this);
    }

    /*!
     * \brief storeLambda, calls storeLambda on p1 and p2 proximities' storeLambda
     * \param cParams
     * \param res
     * \param lambda
     */
    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::defaulttype::BaseVector* lambda) const {
        for (unsigned i=0;i<m_normals.size();i++) {
            m_detection.first->storeLambda(cParams,res,m_cid,i,lambda);
            m_detection.second->storeLambda(cParams,res,m_cid,i,lambda);
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
            vparams->drawTool()->drawArrow(getFirstProximity()->getPosition(), getFirstProximity()->getPosition() + m_normals.m_dirs[0] * scale, scale * 0.1, helper::types::RGBAColor(1,0,0,1));
//            vparams->drawTool()->drawArrow(m_p2->getPosition(), m_p2->getPosition() - m_normals.m_dirs[0] * scale, scale * 0.1, helper::types::RGBAColor(1,0,0,1));
        }

        if (m_normals.size()>1) {
            vparams->drawTool()->drawArrow(getFirstProximity()->getPosition(), getFirstProximity()->getPosition() + m_normals.m_dirs[1] * scale, scale * 0.1, helper::types::RGBAColor(0,1,0,1));
//            vparams->drawTool()->drawArrow(m_p2->getPosition(), m_p2->getPosition() - m_normals.m_dirs[1] * scale, scale * 0.1, helper::types::RGBAColor(0,1,0,1));
        }

        if (m_normals.size()>2) {
            vparams->drawTool()->drawArrow(getFirstProximity()->getPosition(), getFirstProximity()->getPosition() + m_normals.m_dirs[2] * scale, scale * 0.1, helper::types::RGBAColor(0,0,1,1));
//            vparams->drawTool()->drawArrow(m_p2->getPosition(), m_p2->getPosition() - m_normals.m_dirs[2] * scale, scale * 0.1, helper::types::RGBAColor(0,0,1,1));
        }
    }

    inline collisionAlgorithm::BaseProximity::SPtr getFirstProximity() const {
        return m_detection.first;
    }

    inline collisionAlgorithm::BaseProximity::SPtr getSecondProximity() const {
        return m_detection.second;
    }

    const ConstraintNormal getConstraintNormal() const {
        return m_normals;
    }

    unsigned size() const {
        return m_normals.size();
    }

    unsigned id() const {
        return m_cid;
    }

    void buildConstraintProximityMatrix(int cId, sofa::defaulttype::BaseMatrix * J_from, sofa::defaulttype::BaseMatrix * /*J_dest*/, const bool expand){
        m_detection.first->buildConstraintProximityMatrix(cId, J_from,  1.0, expand);
//        m_detection.first->buildConstraintMatrixJ0(cId, J_dest,  -1.0);
//        std::cout<<"proximity position "<<cId<<" = "<<m_detection.first->getPosition()<<std::endl;
    }

    sofa::core::behavior::MechanicalState<defaulttype::Vec3Types> * getStateFrom() const{
        return m_detection.first->getState();
    }

    sofa::core::behavior::MechanicalState<defaulttype::Vec3Types> * getStateDest() const{
        return m_detection.second->getState();
    }

    void buildConstraintProximityMatrix(sofa::core::behavior::MechanicalState<defaulttype::Vec3Types> *MecState, int cID, sofa::defaulttype::BaseMatrix * J_prox, const bool expand){
        if(MecState == getStateFrom()) {
            m_detection.first->buildConstraintProximityMatrix(cID, J_prox,  1.0, expand);
        }

        if(MecState == getStateDest()) {
            m_detection.second->buildConstraintProximityMatrix(cID, J_prox,  -1.0, expand);
        }
    }


    void buildConstraintProximityMatrixDeriv(core::MultiMatrixDerivId cId, unsigned int constraintId) const {
        helper::vector<defaulttype::Vector3> normals;
        normals.clear();
        normals.push_back(defaulttype::Vector3(1, 1, 1));

        m_detection.first->buildJacobianConstraint(cId, normals,  1.0, constraintId);
        m_detection.second->buildJacobianConstraint(cId, normals, -1.0, constraintId);

    }

    void pushNormalIntoVector(helper::vector<defaulttype::Vec3> * vecN){
        for(int i=0; i<m_normals.size(); i++){
            vecN->push_back(m_normals.m_dirs[i]);
        }
    }

    void pushNormalIntoMatrix(sofa::defaulttype::BaseMatrix * matN, unsigned int cId, unsigned int & dirId){
        for(int i=0; i<m_normals.size(); i++){
            defaulttype::Vector3 n = m_normals.m_dirs[i];
            matN->add(dirId, cId*3, n[0]);
            matN->add(dirId, cId*3+1, n[1]);
            matN->add(dirId, cId*3+2, n[2]);
            dirId++;
        }        
    }

    void UpdateConstraintViolationWithProximityPosition(unsigned  cid, const collisionAlgorithm::PairDetection & detection, defaulttype::Vec3 prox_from, bool getF, defaulttype::Vec3 prox_dest, bool getD, defaulttype::BaseVector * delta) const {
            m_normals.UpdateConstraintViolationWithProximityPosition(cid, detection, prox_from, getF, prox_dest, getD, delta);
    }

    void buildConstraintMatrix_alt(core::MultiMatrixDerivId cId, unsigned int & constraintId) const {
        m_cid = constraintId;
        std::cout<<"buildConstraintMatrix_alt"<<std::endl;

        helper::vector<defaulttype::Vector3> dirs;
        dirs.clear();
        for(int i=0; i<m_normals.size(); i++){
            dirs.push_back(defaulttype::Vector3(1,1,1));
        }

        m_detection.first->buildJacobianConstraint(cId, dirs,  1.0, constraintId);
        m_detection.second->buildJacobianConstraint(cId, dirs, -1.0, constraintId);

        constraintId += m_normals.size();
    }


 protected:
    collisionAlgorithm::PairDetection m_detection;
    ConstraintNormal m_normals;
    ResolutionCreator m_creator;
    mutable unsigned m_cid;
};

}

}
