#ifndef SOFA_COMPONENT_CONSTRAINT_COLLISIONALGORITHMS_H
#define SOFA_COMPONENT_CONSTRAINT_COLLISIONALGORITHMS_H

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
#include "ConstraintNormal.h"

namespace sofa {

namespace core {

namespace behavior {


class CollisionAlgorithm : public core::BehaviorModel {
public:

    typedef std::pair<ConstraintProximityPtr,ConstraintProximityPtr> PariProximity;
    typedef helper::vector<PariProximity > PariProximityVector;

    virtual void processAlgorithm(helper::vector<ConstraintNormalPtr> & cn) = 0;

    void updatePosition(SReal /*dt*/) {
        prepareDetection();
    }

protected:

    virtual void prepareDetection() {}
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
