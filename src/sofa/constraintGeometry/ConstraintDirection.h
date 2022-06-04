#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <sofa/constraintGeometry/InternalConstraint.h>

namespace sofa::constraintGeometry {

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseConstraint
 */
class ConstraintDirection : public sofa::core::objectmodel::BaseObject {
public:
    SOFA_CLASS(ConstraintDirection, sofa::core::objectmodel::BaseObject);

    virtual ConstraintNormal createConstraintsNormal(const ConstraintProximity::SPtr & first, const ConstraintProximity::SPtr & second) const = 0;

};

}
