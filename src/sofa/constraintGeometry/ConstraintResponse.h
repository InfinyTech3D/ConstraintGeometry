#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <sofa/constraintGeometry/InternalConstraint.h>

namespace sofa {

namespace constraintGeometry {

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseConstraint
 */
class ConstraintResponse : public sofa::core::objectmodel::BaseObject {
public:
    SOFA_CLASS(ConstraintResponse, sofa::core::objectmodel::BaseObject);

    virtual void createResponseDirections(ConstraintNormal & /*cst*/) {} // by default nothing to do

    virtual core::behavior::ConstraintResolution* createConstraintResolution(const BaseInternalConstraint * cst) const = 0;

};

}

}
