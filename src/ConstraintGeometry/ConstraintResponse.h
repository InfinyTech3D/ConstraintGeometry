#pragma once

#include <CollisionAlgorithm/BaseAlgorithm.h>
#include <CollisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseLagrangianConstraint.h>
#include <ConstraintGeometry/ConstraintNormal.h>
#include <ConstraintGeometry/InternalConstraint.h>

namespace sofa {

namespace constraintGeometry {

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseLagrangianConstraint
 */
class ConstraintResponse : public sofa::core::objectmodel::BaseObject {
public:
    SOFA_CLASS(ConstraintResponse, sofa::core::objectmodel::BaseObject);

    virtual void createResponseDirections(ConstraintNormal & /*cst*/) {} // by default nothing to do

    virtual core::behavior::ConstraintResolution* createConstraintResolution(const BaseInternalConstraint * cst) const = 0;

};

}

}
