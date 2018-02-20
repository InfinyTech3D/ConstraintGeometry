#ifndef SOFA_COMPONENT_CONSTRAINT_ConstraintResponse_H
#define SOFA_COMPONENT_CONSTRAINT_ConstraintResponse_H

#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/PairInteractionConstraint.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/core/topology/BaseTopology.h>
#include <sofa/core/collision/Pipeline.h>
#include <SofaBaseTopology/PointSetTopologyContainer.h>
#include <SofaBaseTopology/EdgeSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
//#include "ConstraintResponse.h"
#include "CollisionAlgorithm.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/gl/template.h>


namespace sofa {

namespace core {

namespace behavior {

class ConstraintResponse {
public:
    virtual void buildConstraintMatrix(const ConstraintParams* cParams, core::MultiMatrixDerivId cId, unsigned cline) = 0;

    virtual void getConstraintViolation(const ConstraintParams* /*cParams*/, defaulttype::BaseVector *v,unsigned cid) = 0;

    virtual core::behavior::ConstraintResolution * getResolution() = 0;

    virtual unsigned size() = 0;
};

typedef std::shared_ptr<ConstraintResponse> ConstraintResponsePtr;

class Constraint : public sofa::core::behavior::BaseConstraint {
public :

    SOFA_CLASS(Constraint, sofa::core::behavior::BaseConstraint );

    virtual void fillConstraints(helper::vector<ConstraintResponsePtr> & cst) = 0;

    void processGeometricalData() {
        m_constraints.clear();
        fillConstraints(m_constraints);
    }

    void buildConstraintMatrix(const ConstraintParams* cParams, core::MultiMatrixDerivId cId, unsigned int &constraintId) {
        for (unsigned i=0;i<m_constraints.size();i++) {
            m_constraints[i]->buildConstraintMatrix(cParams,cId,constraintId);
            constraintId+=m_constraints[i]->size();
        }
    }

    void getConstraintViolation(const ConstraintParams* cParams, defaulttype::BaseVector *v,unsigned cid) {
        for (unsigned i=0;i<m_constraints.size();i++) {
            m_constraints[i]->getConstraintViolation(cParams,v,cid);
            cid+=m_constraints[i]->size();
        }
    }

    void getConstraintResolution(const ConstraintParams* /*cParams*/, std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned int& offset) {
        for (unsigned i=0;i<m_constraints.size();i++) {
            resTab[offset] = m_constraints[i]->getResolution();
            offset+=m_constraints[i]->size();
        }
    }

    void updateForceMask() {}

private:
    helper::vector<ConstraintResponsePtr> m_constraints;

};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
