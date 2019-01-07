#pragma once

#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/constraintGeometry/BaseResponse.h>

namespace sofa {

namespace constraintGeometry {

class Constraint : public sofa::core::behavior::BaseConstraint {
public:
    SOFA_CLASS(Constraint, sofa::core::behavior::BaseConstraint);

    Data<double> d_drawScale;

    Constraint()
    : d_drawScale(initData(&d_drawScale, 1.0, "draw_scale", "draw scale"))
    , l_detection(initLink("detection", "Link to Detection Algorithm"))
    , l_response(initLink("response", "Link to Response")) {}

    void processGeometricalData() {
        //each component of this vector will be deleted by sofa at each time step so we don't have to delete each component
        m_constraints.clear();

        for (unsigned i=0;i<l_detection->getOutput().size();i++) {
            m_constraints.push_back(l_response->createConstraint(l_detection->getOutput()[i]));
        }
    }

    void buildConstraintMatrix(const core::ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned int &constraintId) {
        m_cid = constraintId;
        for (unsigned i=0;i<m_constraints.size();i++) m_constraints[i].buildConstraintMatrix(cId,constraintId);
    }

    void getConstraintViolation(const core::ConstraintParams* /*cParams*/, defaulttype::BaseVector *v,unsigned cid) {
        cid = m_cid;
        for (unsigned i=0;i<m_constraints.size();i++) m_constraints[i].getConstraintViolation(v, cid);
    }

    void getConstraintResolution(const core::ConstraintParams* /*cParams*/, std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned int& offset) {
        for (unsigned i=0;i<m_constraints.size();i++) m_constraints[i].getConstraintResolution(resTab,offset);
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowInteractionForceFields()) return;

        for (unsigned i=0;i<m_constraints.size();i++) m_constraints[i].draw(vparams,d_drawScale.getValue());
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::defaulttype::BaseVector* lambda) {
        if (! cParams) return;

        std::set<sofa::core::behavior::BaseMechanicalState*> processed_states;

        for (unsigned i=0;i<m_constraints.size();i++) {
            if (processed_states.find(m_constraints[i].getFirstProximity()->getState()) == processed_states.end()) {
                m_constraints[i].getFirstProximity()->storeLambda(cParams, res, lambda);
                processed_states.insert(m_constraints[i].getFirstProximity()->getState());
            }

            if (processed_states.find(m_constraints[i].getSecondProximity()->getState()) == processed_states.end()) {
                m_constraints[i].getSecondProximity()->storeLambda(cParams, res, lambda);
                processed_states.insert(m_constraints[i].getSecondProximity()->getState());
            }
        }
    }

    void updateForceMask() {}

protected:    
    unsigned m_cid;
    std::vector<BaseResponse::InternalConstraint> m_constraints;
    core::objectmodel::SingleLink<Constraint,collisionAlgorithm::BaseCollisionAlgorithm,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_detection;
    core::objectmodel::SingleLink<Constraint,BaseResponse,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_response;

};

}

}
