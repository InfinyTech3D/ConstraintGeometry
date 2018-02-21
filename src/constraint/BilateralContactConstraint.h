#ifndef SOFA_COMPONENT_CONSTRAINT_BILATERALCONTACTCONSTRAINT_H
#define SOFA_COMPONENT_CONSTRAINT_BILATERALCONTACTCONSTRAINT_H

#include "ConstraintResponse.h"
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
#include "CollisionAlgorithm.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/gl/template.h>


namespace sofa {

namespace core {

namespace behavior {

class BilateralContactConstraint : public Constraint {
public :

    Data<std::string> d_algo;

    SOFA_CLASS(BilateralContactConstraint, sofa::core::behavior::BaseConstraint );

    BilateralContactConstraint();

    void init();

    void fillConstraints(helper::vector<ConstraintResponsePtr> & cst);

protected:
    CollisionAlgorithm * m_algo;
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
