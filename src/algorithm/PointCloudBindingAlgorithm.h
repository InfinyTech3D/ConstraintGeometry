#ifndef SOFA_COMPONENT_CONSTRAINT_BINDPOINTNALGORITHM_H
#define SOFA_COMPONENT_CONSTRAINT_BINDPOINTNALGORITHM_H

#include "CollisionAlgorithm.h"
#include "ConstraintProximity.h"
#include "geometry/PointGeometry.h"
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>

namespace sofa {

namespace core {

namespace behavior {

class PointCloudBindingAlgorithm : public CollisionAlgorithm {
public:

    SOFA_CLASS(PointCloudBindingAlgorithm , CollisionAlgorithm );

    Data<std::string> d_from;
    Data<std::string> d_dest;
    Data<double> d_maxDist;

    PointCloudBindingAlgorithm();

    void init();

    void processAlgorithm(helper::vector<ConstraintNormalPtr> & cn) ;

protected:
    PointGeometry * m_from;
    PointGeometry * m_dest;
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
