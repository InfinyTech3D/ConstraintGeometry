#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/collisionAlgorithm/BaseGeometryAlgorithm.h>
#include <sofa/constraintGeometry/ConstraintNormal.h>

namespace sofa {

namespace constraintGeometry {

class BaseDirection : public sofa::core::objectmodel::BaseObject {
public:
    SOFA_ABSTRACT_CLASS(BaseDirection, sofa::core::objectmodel::BaseObject);

    // create the direction of the constraint based on the pair detection d
    // size is the desired size (number of constraints)
    virtual ConstraintNormal createConstraintNormal(unsigned size, const collisionAlgorithm::DetectionOutput::PairDetection & d) = 0;

    // create the direction of the constraint based on the pair detection d
    // The BaseDirection must choose the size
    virtual ConstraintNormal createConstraintNormal(const collisionAlgorithm::DetectionOutput::PairDetection & d) = 0;
};

}

}
