#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <sofa/constraintGeometry/InternalConstraint.h>
#include <sofa/constraintGeometry/ConstraintResponse.h>
#include <sofa/constraintGeometry/ConstraintDirection.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>

namespace sofa ::constraintGeometry {

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseConstraint
 */
class BaseNormalHandler : public collisionAlgorithm::CollisionComponent {
public:
    SOFA_ABSTRACT_CLASS(BaseNormalHandler, collisionAlgorithm::CollisionComponent);

//    virtual bool getNormal(collisionAlgorithm::BaseProximity::SPtr prox,type::Vector3 & N) = 0;

    virtual ConstraintProximity::SPtr buildConstraintProximity(collisionAlgorithm::BaseProximity::SPtr prox) = 0;

    virtual const std::type_info & getTypeInfo() = 0;

};

}
