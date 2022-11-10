#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <sofa/constraintGeometry/InternalConstraint.h>
#include <sofa/constraintGeometry/ConstraintResponse.h>
#include <sofa/constraintGeometry/ConstraintDirection.h>

namespace sofa::constraintGeometry {

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseConstraint
 */
template<class FIRST = collisionAlgorithm::BaseBaseProximity, class SECOND = collisionAlgorithm::BaseBaseProximity>
class BaseConstraint : public sofa::core::behavior::BaseConstraint {
public:
    SOFA_CLASS(BaseConstraint, sofa::core::behavior::BaseConstraint);

    typedef collisionAlgorithm::BaseProximity BaseProximity;

    Data<double> d_drawScale;
    Data<bool> d_draw;
    Data<collisionAlgorithm::DetectionOutput<FIRST,SECOND> > d_input; // THIS SHOULD BE REPLACED BY A PAIR OF CST PROXIMITY INPUT

    BaseConstraint()
        : d_drawScale(initData(&d_drawScale, 1.0, "draw_scale", "draw scale"))
        , d_draw(initData(&d_draw, true, "draw", "draw "))
        , d_input(initData(&d_input, "input", "Link to detection output"))
    {}

    virtual ConstraintNormal createConstraintNormal(const typename FIRST::SPtr & first, const typename SECOND::SPtr & second) const = 0;

    virtual core::behavior::ConstraintResolution* createConstraintResolution(const BaseInternalConstraint * cst) const = 0;

    /*!
     * \brief processGeometricalData
     * Clears constraint container and recreates constraints
     */
    virtual void processGeometricalData() {
        m_container.clear();//clear previsou constraints

        auto & input = d_input.getValue();
        for (unsigned i=0;i<input.size();i++) {
            ConstraintNormal CN = createConstraintNormal(input[i].first,input[i].second);

            if (CN.size() == 0) continue;

            InternalConstraint constraint(input[i].first,input[i].second,CN,
                                                        std::bind(&BaseConstraint::createConstraintResolution, this, std::placeholders::_1));


            m_container.push_back(constraint);
        }
    }

    /*!
     * \brief buildConstraintMatrix loops through the constaint container and builds constraint matrix
     * \param cId
     * \param constraintId
     */
    virtual void buildConstraintMatrix(const core::ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned int &constraintId) {
        setConstraintId(constraintId);
        m_nbConstraints = 0;
        for (unsigned i=0;i<m_container.size();i++)
        {
            m_container[i].buildConstraintMatrix(cId,constraintId);
            m_nbConstraints += m_container[i].size();

        }
    }

    /*!
     * \brief getConstraintViolation gets constraint violation for each constraint in constraint container
     * \param v
     */
    virtual void getConstraintViolation(const core::ConstraintParams* /*cParams*/, linearalgebra::BaseVector *v,unsigned /*cid*/) {
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
        if(!d_draw.getValue()) return;
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
                    for (unsigned j=0; j<m_container[i].getPairs().size(); j++) {
                        glVertex3dv(m_container[i].getPairs()[j].first->getPosition().data());
                        glVertex3dv(m_container[i].getPairs()[j].second->getPosition().data());
                    }
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
    virtual void storeLambda(const InternalConstraint & cst, const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::linearalgebra::BaseVector* lambda) {
        cst.storeLambda(cParams,res,lambda);
    }

    /*!
     * \brief storeLambda for each constraint in container
     * \param cParams
     * \param res
     * \param lambda
     */
     virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::linearalgebra::BaseVector* lambda) {
        if (! cParams)
            return;

        for (unsigned i=0;i<m_container.size();i++)
            storeLambda(m_container[i],cParams,res,lambda);
    }

    void updateForceMask() {}

    unsigned getCId ()
    {
        return this->m_cId;
    }

    unsigned getSize ()
    {
        return m_nbConstraints;
    }


protected:
    std::vector<InternalConstraint>  m_container;
    unsigned m_nbConstraints;
};

}
