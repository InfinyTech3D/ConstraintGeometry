#pragma once

#include <sofa/constraintGeometry/ConstraintResponse.h>
#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/objectmodel/DataLink.h>

namespace sofa {

namespace constraintGeometry {

class ContactReponse : public ConstraintResponse {
public:

    ContactReponse() {}

    virtual void addConstraint(ConstraintNormal & normal) {}

    virtual void getConstraintViolation(const defaulttype::Vector3 & N, collisionAlgorithm::ConstraintProximity::SPtr p1, collisionAlgorithm::ConstraintProximity::SPtr p2) const {}

    virtual core::behavior::ConstraintResolution * getResolution() {}
};

}

}
