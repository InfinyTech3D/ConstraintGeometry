#pragma once

#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/constraintGeometry/operations/ConstraintProximityOperation.h>

namespace sofa ::constraintGeometry {

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseConstraint
 */
class BaseNormalHandler : public collisionAlgorithm::CollisionComponent {
public:
    SOFA_ABSTRACT_CLASS(BaseNormalHandler, collisionAlgorithm::CollisionComponent);

    typedef collisionAlgorithm::BaseGeometry BaseGeometry;
    typedef collisionAlgorithm::BaseProximity BaseProximity;

    void init() {
        if (getGeometry()==NULL) {
            std::cerr << "Error cannot find the geometry" << std::endl;
            return;
        }

        getGeometry()->addSlave(this);
    }

    virtual BaseGeometry * getGeometry() = 0;

    virtual const std::type_info & getTypeInfo() = 0;

    virtual ConstraintProximity::SPtr createConstraintProximity(const BaseProximity::SPtr & prox) {
        ConstraintProximityOperation::FUNC operation = ConstraintProximityOperation::get(getTypeInfo(),prox->getTypeInfo());
        return operation(this, prox);
    }

};

}
