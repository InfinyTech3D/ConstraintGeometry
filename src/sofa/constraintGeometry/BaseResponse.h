#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <memory>

namespace sofa {

namespace constraintGeometry {

class ConstraintNormal;

class BaseResponse : public core::objectmodel::BaseObject {
public:

    virtual unsigned size() = 0;

    virtual core::behavior::ConstraintResolution* getConstraintResolution() = 0;

};

}

}

