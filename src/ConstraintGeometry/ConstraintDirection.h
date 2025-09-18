#pragma once

#include <CollisionAlgorithm/BaseAlgorithm.h>
#include <CollisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseLagrangianConstraint.h>
#include <ConstraintGeometry/ConstraintNormal.h>
#include <ConstraintGeometry/InternalConstraint.h>

namespace sofa::constraintGeometry {

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseLagrangianConstraint
 */
class ConstraintDirection : public sofa::core::objectmodel::BaseObject {
public:
    SOFA_CLASS(ConstraintDirection, sofa::core::objectmodel::BaseObject);

    typedef collisionalgorithm::BaseProximity BaseProximity;

    virtual ConstraintNormal createConstraintsNormal(const BaseProximity::SPtr & first, const BaseProximity::SPtr & second) const = 0;

};

}
