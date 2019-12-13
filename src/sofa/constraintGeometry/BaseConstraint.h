#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <sofa/constraintGeometry/InternalConstraint.h>
#include <sofa/constraintGeometry/ConstraintResponse.h>
#include <sofa/constraintGeometry/ConstraintDirection.h>

namespace sofa {

namespace constraintGeometry {

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseConstraint
 */
class BaseConstraint : public sofa::core::behavior::BaseConstraint {
public:
    SOFA_CLASS(BaseConstraint, sofa::core::behavior::BaseConstraint);

    Data<double> d_drawScale;
    Data<collisionAlgorithm::DetectionOutput> d_input;

    BaseConstraint()
        : d_drawScale(initData(&d_drawScale, 1.0, "draw_scale", "draw scale"))
        , d_input(initData(&d_input, "input", "Link to detection output"))
    {}

    virtual ConstraintNormal createConstraintNormal(const collisionAlgorithm::DetectionOutput::PairDetection & detection) const = 0;

    virtual core::behavior::ConstraintResolution* createConstraintResolution(const InternalConstraint & cst) const = 0;

    /*!
     * \brief processGeometricalData
     * Clears constraint container and recreates constraints
     */
    virtual void processGeometricalData() {
        m_container.clear();//clear previsou constraints

        const collisionAlgorithm::DetectionOutput & input = d_input.getValue();
        for (unsigned i=0;i<input.size();i++) {
            ConstraintNormal CN = createConstraintNormal(input[i]);

            if (CN.size() == 0) continue;

            m_container.push_back(InternalConstraint(input[i],CN,
                                                     std::bind(&BaseConstraint::createConstraintResolution, this, std::placeholders::_1)));
        }
    }

    /*!
     * \brief buildConstraintMatrix loops through the constaint container and builds constraint matrix
     * \param cId
     * \param constraintId
     */
    virtual void buildConstraintMatrix(const core::ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned int &constraintId) {
        for (unsigned i=0;i<m_container.size();i++)
            m_container[i].buildConstraintMatrix(cId,constraintId);
    }

    /*!
     * \brief getConstraintViolation gets constraint violation for each constraint in constraint container
     * \param v
     */
    virtual void getConstraintViolation(const core::ConstraintParams* /*cParams*/, defaulttype::BaseVector *v,unsigned /*cid*/) {
        for (unsigned i=0;i<m_container.size();i++)
            m_container[i].getConstraintViolation(v);
    }

    /*!
     * \brief getConstraintResolution creates constraint resolution for each constaint in container
     * \param resTab
     * \param offset
     */
    virtual void getConstraintResolution(const core::ConstraintParams* /*cParams*/, std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned int& offset) {
        for (unsigned i=0;i<m_container.size();i++) {
            resTab[offset] = m_container[i].createConstraintResolution();
            offset += m_container[i].size();
        }
    }

    void draw(const core::visual::VisualParams* vparams) {
        if(d_drawScale.getValue()>std::numeric_limits<double>::min())
        {
            if (vparams->displayFlags().getShowInteractionForceFields()) {
                for (unsigned i=0;i<m_container.size();i++)
                    m_container[i].draw(vparams,d_drawScale.getValue());
            }

            if (vparams->displayFlags().getShowCollisionModels()) {
                glDisable(GL_LIGHTING);
                glColor4f(0,1,0,1);

                glBegin(GL_LINES);
                for (unsigned i=0;i<m_container.size();i++) {
                    glVertex3dv(m_container[i].getFirstProximity()->getPosition().data());
                    glVertex3dv(m_container[i].getSecondProximity()->getPosition().data());
                }
                glEnd();
            }
        }
    }

    /*!
     * \brief storeLambda, virtual method called for each container in storeLambda
     * \param cst : Internal Constraint Shared Pointer (typedef in Internal Constraint)
     * \param cParams
     * \param res
     * \param lambda
     */
    virtual void storeLambda(const InternalConstraint & cst, const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::defaulttype::BaseVector* lambda) {
        cst.storeLambda(cParams,res,lambda);
    }

    /*!
     * \brief storeLambda for each constraint in container
     * \param cParams
     * \param res
     * \param lambda
     */
     virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::defaulttype::BaseVector* lambda) {
        if (! cParams)
            return;

        for (unsigned i=0;i<m_container.size();i++)
            storeLambda(m_container[i],cParams,res,lambda);
    }

    void updateForceMask() {}

protected:
    std::vector<InternalConstraint>  m_container;
};

}

}
