#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <sofa/constraintGeometry/InternalConstraint.h>
#include <sofa/constraintGeometry/ConstraintResponse.h>
#include <sofa/constraintGeometry/ConstraintDirection.h>

namespace sofa ::constraintGeometry {

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseConstraint
 */
class BaseNormalHandler : public sofa::core::objectmodel::BaseObject {
public:
    SOFA_ABSTRACT_CLASS(BaseNormalHandler, sofa::core::objectmodel::BaseObject);

    virtual type::Vector3 getNormal(collisionAlgorithm::BaseProximity::SPtr prox) = 0;

};

}
