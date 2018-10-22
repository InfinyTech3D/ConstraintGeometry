#pragma once

#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/constraintGeometry/ConstraintResponse.h>
#include <sofa/core/objectmodel/DataLink.h>
#include <sofa/core/behavior/BaseConstraint.h>

namespace sofa {

namespace constraintGeometry {

class GenericConstraint : public sofa::core::behavior::BaseConstraint {
public:    
    DataLink<ConstraintResponse> d_response;

    GenericConstraint()
    : d_response(initData(&d_response, "response", "Response")) {}

    void processGeometricalData() {
        m_constraints.clear();
        d_response->createResponse(m_constraints);
    }

    void buildConstraintMatrix(const core::ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned int &constraintId) {
        for (unsigned i=0;i<m_constraints.size();i++) {
            m_constraints[i].buildConstraintMatrix(cId,constraintId);
            constraintId += m_constraints[i].size();
        }
    }

    void getConstraintViolation(const core::ConstraintParams* /*cParams*/, defaulttype::BaseVector *v,unsigned cid) {
        for (unsigned i=0;i<m_constraints.size();i++) {
            m_constraints[i].getConstraintViolation(v, cid);
            cid+=m_constraints[i].size();
        }
    }

    void getConstraintResolution(const core::ConstraintParams* /*cParams*/, std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned int& offset) {
        for (unsigned i=0;i<m_constraints.size();i++) {
            resTab[offset] = m_constraints[i].getConstraintResolution();
            offset+=m_constraints[i].size();
        }
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowInteractionForceFields()) return;

        for (unsigned i=0;i<m_constraints.size();i++) m_constraints[i].draw(vparams);
    }

    void updateForceMask() {}

    virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::defaulttype::BaseVector* lambda) {}

protected:
    helper::vector<ConstraintResponse::ConstraintNormal> m_constraints;
};

}

}
