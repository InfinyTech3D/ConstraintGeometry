#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/constraintGeometry/normals/ConstraintNormal.h>
#include <sofa/constraintGeometry/InternalConstraint.h>

namespace sofa {

namespace constraintGeometry {

/*!
 * \brief The ConstraintContainer class is a generic Constraint Container
 */
class ConstraintContainer {
public:

    /*!
     * \brief clears constraint vector
     */
    void clear() {
        m_constraints.clear();
    }

    const InternalConstraint::SPtr & operator[](int i) const {
        return m_constraints[i];
    }

    unsigned size() const {
        return m_constraints.size();
    }

    /*!
     * \fn push_back
     * \brief adds a constraint to the list of constraints
     */
    template<class FwdObject,class FwdFunction>
    void push_back(const FwdObject * obj, const collisionAlgorithm::PairDetection & d, const ConstraintNormal & N, FwdFunction f) {
        m_constraints.push_back(
            InternalConstraint::SPtr(new InternalConstraint(
                d.first,
                d.second,
                N,
                std::unique_ptr<InternalConstraint::ResolutionCreator>(
                    new InternalConstraint::ResolutionCreatorImpl<FwdObject,FwdFunction>(obj,f)
                )
            ))
        );
    }

    /*!
     * \fn push_back
     * \brief adds a constraint to the list of constraints
     */
    template<class FwdObject,class FwdFunction>
    void push_back(const FwdObject * obj, collisionAlgorithm::BaseProximity::SPtr p1,collisionAlgorithm::BaseProximity::SPtr p2, const ConstraintNormal & N, FwdFunction f) {
        m_constraints.push_back(
            InternalConstraint::SPtr(new InternalConstraint(
                p1,
                p2,
                N,
                std::unique_ptr<InternalConstraint::ResolutionCreator>(
                    new InternalConstraint::ResolutionCreatorImpl<FwdObject,FwdFunction>(obj,f)
                )
            ))
        );
    }

protected:
    std::vector<InternalConstraint::SPtr> m_constraints;
};

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseConstraint
 */
class BaseConstraint : public sofa::core::behavior::BaseConstraint {
public:
    SOFA_CLASS(BaseConstraint, sofa::core::behavior::BaseConstraint);

    Data<double> d_drawScale;

    BaseConstraint()
    : d_drawScale(initData(&d_drawScale, 1.0, "draw_scale", "draw scale")) {}

    virtual void createConstraints(ConstraintContainer & constraints) = 0;

    /*!
     * \brief processGeometricalData
     * Clears constraint container and recreates constraints
     */
    void processGeometricalData() {
        m_container.clear();
        createConstraints(m_container);
    }

    /*!
     * \brief buildConstraintMatrix loops through the constaint container and builds constraint matrix
     * \param cId
     * \param constraintId
     */
    virtual void buildConstraintMatrix(const core::ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned int &constraintId) {
        for (unsigned i=0;i<m_container.size();i++)
            m_container[i]->buildConstraintMatrix(cId,constraintId);
    }

    /*!
     * \brief getConstraintViolation gets constraint violation for each constraint in constraint container
     * \param v
     */
    virtual void getConstraintViolation(const core::ConstraintParams* /*cParams*/, defaulttype::BaseVector *v,unsigned /*cid*/) {
        for (unsigned i=0;i<m_container.size();i++)
            m_container[i]->getConstraintViolation(v);
    }

    /*!
     * \brief getConstraintResolution creates constraint resolution for each constaint in container
     * \param resTab
     * \param offset
     */
    virtual void getConstraintResolution(const core::ConstraintParams* /*cParams*/, std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned int& offset) {
        for (unsigned i=0;i<m_container.size();i++) {
            resTab[offset] = m_container[i]->createConstraintResolution();
            offset += m_container[i]->size();
        }
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (vparams->displayFlags().getShowInteractionForceFields()) {
            for (unsigned i=0;i<m_container.size();i++)
                m_container[i]->draw(vparams,d_drawScale.getValue());
        }

        if (vparams->displayFlags().getShowCollisionModels()) {
            glDisable(GL_LIGHTING);
            glColor4f(0,1,0,1);

            glBegin(GL_LINES);
            for (unsigned i=0;i<m_container.size();i++) {
                glVertex3dv(m_container[i]->getFirstProximity()->getPosition().data());
                glVertex3dv(m_container[i]->getSecondProximity()->getPosition().data());
            }
            glEnd();
        }
    }

    /*!
     * \brief storeLambda, virtual method called for each container in storeLambda
     * \param cst : Internal Constraint Shared Pointer (typedef in Internal Constraint)
     * \param cParams
     * \param res
     * \param lambda
     */
    virtual void storeLambda(InternalConstraint::SPtr cst, const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::defaulttype::BaseVector* lambda) {
        cst->storeLambda(cParams,res,lambda);
    }

    /*!
     * \brief storeLambda for each constraint in container
     * \param cParams
     * \param res
     * \param lambda
     */
    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::defaulttype::BaseVector* lambda) {
        if (! cParams)
            return;

        for (unsigned i=0;i<m_container.size();i++)
            storeLambda(m_container[i],cParams,res,lambda);
    }

    void updateForceMask() {}

protected:
    ConstraintContainer m_container;
};

}

}
