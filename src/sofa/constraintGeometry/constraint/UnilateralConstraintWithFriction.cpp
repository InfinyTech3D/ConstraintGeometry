#include <sofa/constraintGeometry/constraint/UnilateralConstraintWithFriction.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int UnilateralConstraintWithFrictionClass = core::RegisterObject("UnilateralConstraintWithFriction")
.add< UnilateralConstraintWithFriction >()
.addAlias("ConstraintUFF");

}

}

