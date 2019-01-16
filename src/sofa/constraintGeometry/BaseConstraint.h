#pragma once

#include <sofa/collisionAlgorithm/BaseGeometryAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <sofa/constraintGeometry/InternalConstraint.h>

namespace sofa {

namespace constraintGeometry {

class BaseConstraint : public sofa::core::behavior::BaseConstraint {
public:
    SOFA_CLASS(BaseConstraint, sofa::core::behavior::BaseConstraint);

    Data<double> d_drawScale;
    Data<collisionAlgorithm::DetectionOutput> d_input;

    BaseConstraint()
    : d_drawScale(initData(&d_drawScale, 1.0, "draw_scale", "draw scale"))
    , d_input(initData(&d_input, "input" , "this")) {}

    virtual InternalConstraint createConstraint(const collisionAlgorithm::DetectionOutput::PairDetection & out) = 0;

    void processGeometricalData() {
        //each component of this vector will be deleted by sofa at each time step so we don't have to delete each component
        m_constraints.clear();

        for (unsigned i=0;i<d_input.getValue().size();i++) {
            m_constraints.push_back(createConstraint(d_input.getValue()[i]));
        }
    }

    virtual void buildConstraintMatrix(const core::ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned int &constraintId) {
        for (unsigned i=0;i<m_constraints.size();i++) m_constraints[i].buildConstraintMatrix(cId,constraintId);
    }

    virtual void getConstraintViolation(const core::ConstraintParams* /*cParams*/, defaulttype::BaseVector *v,unsigned /*cid*/) {
        for (unsigned i=0;i<m_constraints.size();i++) m_constraints[i].getConstraintViolation(v);
    }

    virtual void getConstraintResolution(const core::ConstraintParams* /*cParams*/, std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned int& offset) {
        for (unsigned i=0;i<m_constraints.size();i++) m_constraints[i].getConstraintResolution(resTab,offset);
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowInteractionForceFields()) return;

        for (unsigned i=0;i<m_constraints.size();i++) m_constraints[i].draw(vparams,d_drawScale.getValue());
    }

    virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::defaulttype::BaseVector* lambda) {
        if (! cParams) return;

        for (unsigned i=0;i<m_constraints.size();i++) m_constraints[i].storeLambda(cParams,res,lambda);
    }

    void updateForceMask() {}

protected:    
    std::vector<InternalConstraint> m_constraints;
};

}

}
