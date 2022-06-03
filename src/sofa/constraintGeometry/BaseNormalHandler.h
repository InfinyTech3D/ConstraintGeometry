#pragma once

#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>

namespace sofa ::constraintGeometry {

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseConstraint
 */
class BaseNormalHandler : public collisionAlgorithm::CollisionComponent {
public:
    SOFA_ABSTRACT_CLASS(BaseNormalHandler, collisionAlgorithm::CollisionComponent);

    virtual const std::type_info & getTypeInfo() = 0;

};

}
