#ifndef SOFA_COMPONENT_CONSTRAINT_COLLISIONDETECTIONALGORITHM_H
#define SOFA_COMPONENT_CONSTRAINT_COLLISIONDETECTIONALGORITHM_H

#include "ConstraintProximity.h"
#include "CollisionAlgorithm.h"
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/helper/gl/template.h>

namespace sofa {

namespace core {

namespace behavior {

class CollisionDetectionAlgorithm : public CollisionAlgorithm {
public:

    SOFA_CLASS(CollisionDetectionAlgorithm , CollisionAlgorithm );

    Data<std::string> d_from;
    Data<std::string> d_dest;
    Data<double> d_maxf;

    CollisionDetectionAlgorithm();

    void init();

    void processAlgorithm(helper::vector<ConstraintNormalPtr> & cn) ;

private:

    PariProximity getClosestPoint(unsigned i);
    BaseGeometry * m_from;
    BaseGeometry * m_dest;

};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
