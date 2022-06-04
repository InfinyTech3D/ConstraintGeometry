#pragma once

#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/constraintGeometry/ConstraintProximityOperation.h>

namespace sofa ::constraintGeometry {

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseConstraint
 */
class BaseNormalHandler : public collisionAlgorithm::CollisionComponent {
public:
    SOFA_ABSTRACT_CLASS(BaseNormalHandler, collisionAlgorithm::CollisionComponent);

    core::objectmodel::SingleLink<BaseNormalHandler, collisionAlgorithm::BaseGeometry, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    BaseNormalHandler()
    : l_geometry(initLink("geometry","link to needle data")) {}

    void init() {
        if (l_geometry==NULL) {
            std::cerr << "Error cannot find the geometry" << std::endl;
            return;
        }

        l_geometry->addSlave(this);
    }

    collisionAlgorithm::BaseGeometry * getGeometry() {
        return l_geometry.get();
    }

    virtual const std::type_info & getTypeInfo() = 0;

    virtual ConstraintProximity::SPtr createConstraintProximity(const collisionAlgorithm::BaseProximity::SPtr & prox) {
        ConstraintProximityOperation::FUNC operation = ConstraintProximityOperation::get(getTypeInfo(),prox->getTypeInfo());
        return operation(this, prox);
    }

};

}
