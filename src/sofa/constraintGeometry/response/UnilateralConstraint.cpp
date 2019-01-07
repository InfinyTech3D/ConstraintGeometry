#include <sofa/constraintGeometry/response/UnilateralConstraint.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int UnilateralConstraintResponseClass = core::RegisterObject("UnilateralConstraintResponse")
.add< UnilateralConstraintResponse >()
.addAlias("ResponseU");

}

}

