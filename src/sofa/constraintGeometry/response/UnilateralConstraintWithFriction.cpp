#include <sofa/constraintGeometry/response/UnilateralConstraintWithFriction.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int UnilateralConstraintWithFrictionResponseClass = core::RegisterObject("UnilateralConstraintWithFrictionResponse")
.add< UnilateralConstraintWithFrictionResponse >()
.addAlias("ResponseUFF");

}

}

